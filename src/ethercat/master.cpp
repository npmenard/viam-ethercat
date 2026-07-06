#include "ethercat/master.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string>
#include <utility>

#include "ethercat/cia402.hpp"

namespace ethercat {

namespace {

constexpr std::uint16_t kModesOfOp = 0x6060;  // CiA402 modes-of-operation (U8): PP=1, PV=3; SDO-set in PRE-OP
constexpr long kNsPerSec = 1'000'000'000L;

std::uint32_t field_key(std::uint16_t index, std::uint8_t sub) noexcept {
    return (static_cast<std::uint32_t>(index) << 8U) | sub;
}

}  // namespace

Master::Master(MasterConfig config, std::unique_ptr<EcatBackend> backend) : config_(std::move(config)), backend_(std::move(backend)) {
    if (!backend_) {
        throw Error("Master: null backend");
    }
    if (config_.ifname.empty()) {
        throw Error("Master: empty interface name");
    }
    if (config_.slaves.empty()) {
        throw Error("Master: no slaves configured");
    }
    if (config_.target_loop_rate_hz == 0 || config_.target_loop_rate_hz > 1000) {
        throw Error("Master: target_loop_rate_hz " + std::to_string(config_.target_loop_rate_hz) + " out of range (1..1000)");
    }
    // Derive the AWAIT_OP bounds once from config, so there is no per-cycle math on the
    // bring-up path. The counts clamp 0 -> 1 (a zero hold-confirm would declare OP on no held
    // evidence; a zero nudge interval would divide by zero). The give-up bound is configured in
    // wall time and converted at the configured rate, so the patience window is rate-independent
    // (e.g. 30 s at 250 Hz = 7'500 cycles).
    op_hold_confirm_cycles_ = std::max(config_.op_hold_confirm_cycles, 1U);
    op_nudge_interval_cycles_ = std::max(config_.op_nudge_interval_cycles, 1U);
    const std::uint64_t await_cycles = (static_cast<std::uint64_t>(config_.op_await_timeout_ms) * config_.target_loop_rate_hz) / 1000ULL;
    op_await_bound_cycles_ = static_cast<std::uint32_t>(std::clamp<std::uint64_t>(await_cycles, 1, UINT32_MAX));

    // Validate the loop rate against each slave's declared SYNC0 cycle granularity up front.
    // Workaround for drives that reject a non-multiple cycle only as a cryptic device fault at
    // OP entry.
    if (config_.use_distributed_clocks) {
        const std::uint64_t cycle_ns = static_cast<std::uint64_t>(kNsPerSec) / config_.target_loop_rate_hz;
        for (const SlaveConfig& sc : config_.slaves) {
            const std::uint32_t g = sc.sync_cycle_granularity_ns;
            if (g == 0 || cycle_ns % g == 0) {
                continue;
            }
            // Suggest the nearest rates (within the validated 1..1000 Hz range) whose truncated
            // cycle is a multiple, scanned with the same arithmetic as the check.
            const auto rate_ok = [&](std::uint32_t r) { return (static_cast<std::uint64_t>(kNsPerSec) / r) % g == 0; };
            std::uint32_t lower = 0;
            for (std::uint32_t r = config_.target_loop_rate_hz; r >= 1; --r) {
                if (rate_ok(r)) {
                    lower = r;
                    break;
                }
            }
            std::uint32_t higher = 0;
            for (std::uint32_t r = config_.target_loop_rate_hz; r <= 1000; ++r) {
                if (rate_ok(r)) {
                    higher = r;
                    break;
                }
            }
            std::string nearest;
            if (lower != 0) {
                nearest += " " + std::to_string(lower) + " Hz";
            }
            if (higher != 0) {
                nearest += std::string(lower != 0 ? " /" : "") + " " + std::to_string(higher) + " Hz";
            }
            throw Error("Master: slave " + std::to_string(sc.slave_id) + " declares sync_cycle_granularity_ns=" + std::to_string(g) +
                        " but target_loop_rate_hz=" + std::to_string(config_.target_loop_rate_hz) + " gives a " + std::to_string(cycle_ns) +
                        " ns SYNC0 cycle that is not a multiple -- the drive would reject it at OP entry. Nearest valid"
                        " rates:" +
                        (nearest.empty() ? " none in 1..1000 Hz" : nearest));
        }
    }
}

void Master::init() {
    const std::size_t count = backend_->open(config_.ifname);
    if (count != config_.slaves.size()) {
        throw Error("EtherCAT bus on '" + config_.ifname + "': found " + std::to_string(count) + " slaves, config expects " +
                    std::to_string(config_.slaves.size()));
    }
}

std::map<std::uint32_t, Master::MappedField> Master::build_field_table(std::uint16_t slave, const PdoMap& map) {
    std::map<std::uint32_t, MappedField> fields;
    std::size_t bit = 0;
    for (const std::uint16_t pdo : map.pdo_indices) {
        const auto it = map.entries.find(pdo);
        if (it == map.entries.end()) {
            throw PdoMappingError("slave " + std::to_string(slave) + ": PDO has no entry list while building field table");
        }
        for (const PdoEntry& e : it->second) {
            if (e.index != 0x0000) {  // 0x0000 = padding/gap: advances the offset, no named field
                if ((bit % 8) != 0) {
                    throw PdoMappingError("slave " + std::to_string(slave) + ": mapped object is not byte-aligned (bit offset " +
                                          std::to_string(bit) + ")");
                }
                if ((e.bit_length % 8) != 0) {
                    throw PdoMappingError("slave " + std::to_string(slave) + ": mapped object width " + std::to_string(e.bit_length) +
                                          " bits is not a whole number of bytes");
                }
                fields[field_key(e.index, e.subindex)] = MappedField{bit / 8, e.bit_length};
            }
            bit += e.bit_length;
        }
    }
    return fields;
}

void Master::configure() {
    // Maps are writable only in PRE-OP and are not stored in EEPROM, so this runs
    // every configure() / power-on.
    backend_->request_state(0, EcatState::PreOp);

    for (const SlaveConfig& sc : config_.slaves) {
        apply_pdo_map(*backend_, sc.slave_id, sc.rxpdo, PdoDirection::Rx);
        apply_pdo_map(*backend_, sc.slave_id, sc.txpdo, PdoDirection::Tx);
        // Set modes-of-operation (0x6060, U8) via SDO; it is not mapped cyclically. A drive
        // left in mode 0 never moves. PP=1 / PV=3, from the configured default_mode.
        const std::array<std::byte, 1> mode{static_cast<std::byte>(static_cast<std::uint8_t>(sc.default_mode))};
        backend_->sdo_write(sc.slave_id, kModesOfOp, 0, mode);
    }

    // DC SYNC0 cycle = loop period (validated against the drive's granularity in the ctor).
    const auto cycle_ns = static_cast<std::uint32_t>(kNsPerSec / static_cast<long>(config_.target_loop_rate_hz));

    if (config_.use_distributed_clocks) {
        backend_->arm_dc_sync(cycle_ns, config_.dc_sync0_shift_ns);
    }

    backend_->map_process_data();
    expected_wkc_ = backend_->expected_wkc();

    slaves_.clear();
    for (const SlaveConfig& sc : config_.slaves) {
        const SlaveInfo info = backend_->slave_info(sc.slave_id);

        // Validate the applied (wire) image against the configured map. If a drive silently
        // rejected part of the remap, map_process_data lays out the drive's default image while
        // the field table (built from config below) carries offsets for the expected map.
        const std::size_t rx_bytes = sc.rxpdo.byte_size();
        const std::size_t tx_bytes = sc.txpdo.byte_size();
        if (info.output_bytes != rx_bytes || info.input_bytes != tx_bytes) {
            throw PdoMappingError("slave " + std::to_string(sc.slave_id) + ": applied RxPDO " + std::to_string(info.output_bytes) +
                                  " B / TxPDO " + std::to_string(info.input_bytes) + " B != configured " + std::to_string(rx_bytes) +
                                  " / " + std::to_string(tx_bytes) + " B (remap did not take)");
        }
        slaves_.emplace_back(sc.slave_id, info.input_bytes);
        SlaveRuntime& rt = slaves_.back();
        rt.io = backend_->slave_io(sc.slave_id);
        rt.rx_fields = build_field_table(sc.slave_id, sc.rxpdo);
        rt.tx_fields = build_field_table(sc.slave_id, sc.txpdo);
    }

    if (config_.use_distributed_clocks) {
        backend_->configure_dc_configdc();
    }

    backend_->request_state(0, EcatState::SafeOp);

    dc_enabled_ = config_.use_distributed_clocks;
    bringup_phase_ = BringupPhase::Settle;
    bringup_settle_count_ = 0;
    bringup_await_count_ = 0;
    bringup_op_hold_streak_ = 0;
    bringup_al_code_ = 0;
    fault_.store(false, std::memory_order_relaxed);
    consecutive_wkc_errors_ = 0;
    settle_remaining_ = 0;
    total_cycles_.store(0, std::memory_order_relaxed);
    bad_cycles_.store(0, std::memory_order_relaxed);
    operational_.store(false, std::memory_order_relaxed);
}

BringupStatus Master::bringup_step(bool drive_sync_faulted, bool drive_present) noexcept {
    const int wkc = backend_->exchange();
    last_wkc_.store(wkc, std::memory_order_relaxed);

    switch (bringup_phase_) {
        case BringupPhase::Settle: {
            std::uint32_t target = 1U;
            if (dc_enabled_ && config_.dc_op_gate_cycles != 0) {
                target = config_.dc_op_gate_cycles;
            }
            if (++bringup_settle_count_ >= target) {
                backend_->set_state(0, EcatState::Op);  // writestate only; this loop pumps the transition
                bringup_await_count_ = 0;
                bringup_op_hold_streak_ = 0;
                bringup_phase_ = BringupPhase::AwaitOp;
            }
            return BringupStatus::Gating;
        }
        case BringupPhase::AwaitOp: {
            ++bringup_await_count_;
            // reset al code
            if (const std::uint16_t al = backend_->al_status_code(1); al != 0) {
                bringup_al_code_ = al;
            }
            if (bringup_await_count_ % op_nudge_interval_cycles_ == 0) {
                backend_->reack_op(0);
            }

            if (wkc == expected_wkc_ && !drive_sync_faulted && drive_present) {
                ++bringup_op_hold_streak_;
            } else {
                bringup_op_hold_streak_ = 0;
            }
            if (bringup_op_hold_streak_ >= op_hold_confirm_cycles_) {
                operational_.store(true, std::memory_order_relaxed);
                settle_remaining_ = dc_enabled_ ? config_.dc_settle_cycles : 0;
                fault_.store(false, std::memory_order_relaxed);
                consecutive_wkc_errors_ = 0;
                bringup_phase_ = BringupPhase::Done;
                return BringupStatus::Operational;
            }
            if (bringup_await_count_ >= op_await_bound_cycles_) {
                bringup_phase_ = BringupPhase::Aborted;
                return BringupStatus::Aborted;
            }
            return BringupStatus::AwaitingOp;
        }
        case BringupPhase::Done:
            return BringupStatus::Operational;
        case BringupPhase::Aborted:
            return BringupStatus::Aborted;
    }
    return BringupStatus::Aborted;  // unreachable; satisfies the compiler
}

void Master::process() noexcept {
    const int wkc = backend_->exchange();
    last_wkc_.store(wkc, std::memory_order_relaxed);
    total_cycles_.fetch_add(1, std::memory_order_relaxed);
    if (wkc < 0 || wkc < expected_wkc_) {
        bad_cycles_.fetch_add(1, std::memory_order_relaxed);
    }
    // Post-OP DC settle grace: while it lasts, fully clear the latch state every cycle, so no
    // bad streak during the grace can carry past the grace boundary and trip a spurious latch
    // the instant it ends. The DC phase PI is still pulling into the window; a few
    // partial-processing cycles are benign.
    const bool in_grace = settle_remaining_ > 0;
    if (in_grace) {
        --settle_remaining_;
        consecutive_wkc_errors_ = 0;
    }
    if (wkc < 0 || wkc < expected_wkc_) {
        if (!in_grace) {
            ++consecutive_wkc_errors_;
            if (consecutive_wkc_errors_ >= config_.max_consecutive_wkc_errors) {
                fault_wkc_.store(wkc, std::memory_order_relaxed);
                fault_.store(true, std::memory_order_release);
                operational_.store(false, std::memory_order_relaxed);
            }
        }
    } else {
        consecutive_wkc_errors_ = 0;
        working_counter_.store(wkc, std::memory_order_relaxed);
    }

    ++cycle_;
    for (SlaveRuntime& rt : slaves_) {
        rt.cache.publish_inputs(rt.io.inputs, static_cast<std::uint16_t>(wkc < 0 ? 0 : wkc), cycle_);
    }
}

void Master::close() noexcept {
    operational_.store(false, std::memory_order_relaxed);
    backend_->close();
}

std::span<std::byte> Master::outputs(std::uint16_t slave) noexcept {
    // rt can be const: SlaveIo::outputs is a std::span<std::byte> (shallow-const),
    // so a const SlaveRuntime still yields a writable view of the command image.
    for (const SlaveRuntime& rt : slaves_) {
        if (rt.slave_id == slave) {
            return rt.io.outputs;
        }
    }
    return {};
}

std::span<const std::byte> Master::input_image(std::uint16_t slave) const noexcept {
    for (const SlaveRuntime& rt : slaves_) {
        if (rt.slave_id == slave) {
            return rt.io.inputs;
        }
    }
    return {};
}

PdoSnapshot Master::read_inputs(std::uint16_t slave) const noexcept {
    for (const SlaveRuntime& rt : slaves_) {
        if (rt.slave_id == slave) {
            return rt.cache.read_inputs();
        }
    }
    return {};
}

Master::SlaveRuntime& Master::runtime_for(std::uint16_t slave) {
    for (SlaveRuntime& rt : slaves_) {
        if (rt.slave_id == slave) {
            return rt;
        }
    }
    throw Error("Master: unknown slave " + std::to_string(slave));
}

const Master::SlaveRuntime& Master::runtime_for(std::uint16_t slave) const {
    for (const SlaveRuntime& rt : slaves_) {
        if (rt.slave_id == slave) {
            return rt;
        }
    }
    throw Error("Master: unknown slave " + std::to_string(slave));
}

PdoCache& Master::cache(std::uint16_t slave) {
    return runtime_for(slave).cache;
}

FieldLocation Master::rx_field(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const {
    const SlaveRuntime& rt = runtime_for(slave);
    const auto it = rt.rx_fields.find(field_key(index, sub));
    if (it == rt.rx_fields.end()) {
        throw PdoMappingError("slave " + std::to_string(slave) + ": object not in the RxPDO (command) map");
    }
    return FieldLocation{it->second.byte_offset};
}

FieldLocation Master::tx_field(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const {
    const SlaveRuntime& rt = runtime_for(slave);
    const auto it = rt.tx_fields.find(field_key(index, sub));
    if (it == rt.tx_fields.end()) {
        throw PdoMappingError("slave " + std::to_string(slave) + ": object not in the TxPDO (feedback) map");
    }
    return FieldLocation{it->second.byte_offset};
}

FieldLocation Master::resolve_field(const std::map<std::uint32_t, MappedField>& table,
                                    std::uint16_t index,
                                    std::uint8_t sub,
                                    std::size_t want_width,
                                    std::uint16_t slave,
                                    bool is_tx) {
    const char* const which = is_tx ? "TxPDO (feedback)" : "RxPDO (command)";
    const auto it = table.find(field_key(index, sub));
    if (it == table.end()) {
        // Not-in-map is a map-membership failure, so throw PdoMappingError; the operator fixes
        // it by adding the object to the map. PdoMappingError spans both apply-time
        // (apply_pdo_map) and this runtime access of an un-mapped object. A wrong-width access
        // throws the base Error below, not PdoMappingError, so try_resolve_rx/try_resolve_tx
        // do not swallow a wrong-width object as absent.
        throw PdoMappingError("slave " + std::to_string(slave) + ": object " + std::to_string(index) + ":" + std::to_string(sub) +
                              " is not in the " + which + " map");
    }
    // Width assertion: the Field's T must match the mapped object's width. Catches a silent
    // wrong-width access (e.g. an int16 alias against a 32-bit-mapped object would read 2 of 4
    // bytes in-bounds, with no throw). Checked against the internal table's bit_length (the
    // public FieldLocation is offset-only); build_field_table guarantees bit_length % 8 == 0.
    const std::size_t mapped_width = it->second.bit_length / 8U;
    if (mapped_width != want_width) {
        throw Error("slave " + std::to_string(slave) + ": object " + std::to_string(index) + ":" + std::to_string(sub) +
                    " width mismatch -- the Field type is " + std::to_string(want_width) + " byte(s) but the object is mapped " +
                    std::to_string(mapped_width) + " byte(s)");
    }
    return FieldLocation{it->second.byte_offset};
}

void Master::sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) {
    // The CoE mailbox transfer runs on the caller's thread and blocks until it completes. It is
    // safe while the RT PDO loop is running: SOEM v2's port is thread-safe (per-index frame
    // buffers plus PRIO_INHERIT getindex/tx/rx mutexes in nicdrv), and the mailbox SyncManager
    // is distinct from the PDO SM, so a one-shot SDO neither corrupts nor (being one-shot, not a
    // tight poll) starves the cyclic LRW.
    backend_->sdo_write(slave, index, sub, data);
}

std::size_t Master::sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) {
    // Caller-thread, blocking, RT-concurrent-safe (see sdo_write). Returns bytes read into `out`.
    return backend_->sdo_read(slave, index, sub, out);
}

std::string Master::last_error() const {
    // Acquire pairs with process()'s release store of fault_, so fault_wkc_ below
    // is the value that was current when the fault latched (never stale).
    if (!fault_.load(std::memory_order_acquire)) {
        return {};
    }
    return "EtherCAT working-counter fault on '" + config_.ifname + "': got " + std::to_string(fault_wkc_.load(std::memory_order_relaxed)) +
           ", expected " + std::to_string(expected_wkc_) + " for " + std::to_string(config_.max_consecutive_wkc_errors) +
           " consecutive cycles";
}

}  // namespace ethercat
