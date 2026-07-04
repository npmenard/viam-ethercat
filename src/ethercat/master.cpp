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
// The AWAIT_OP bounds (hold-confirm, nudge-interval, give-up) live in MasterConfig: they
// carry a wall-time meaning that depends on the loop rate, and are derived once in the ctor.

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

    // Validate the loop rate against each slave's declared SYNC0 cycle granularity up front. A
    // non-multiple cycle otherwise surfaces only as a cryptic fault at OP entry (the A6 Er74.0
    // "cycle error"). Uses the same truncated-cycle arithmetic configure() arms SYNC0 with, so
    // the value checked is the value the drive sees. Granularity is per-slave config data (0 =
    // no constraint); only meaningful when DC/SYNC0 is in play.
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
        // The structural PDO remap (intrinsic to the init->map sequence, not consumer policy):
        // assign 0x1600/0x1A00 to SM2/SM3 (0x1C12/0x1C13) and write the entry lists. Setup-SDO
        // policy is the consumer's, run via Master::sdo_write() post-configure while it is the
        // single port owner.
        //
        // An SM-sync-type write (0x1C32:01 / 0x1C33:01) must go after this apply_pdo_map, not
        // before: several drives re-default 0x1C32 when the PDO assignment changes, so a
        // sync-type write done before the assignment is silently clobbered. The A6 needs none
        // (it self-selects DC from the PRE-OP SYNC0 arm), but a drive that needs an explicit
        // sync-type write must place its hook here, post-remap.
        apply_pdo_map(*backend_, sc.slave_id, sc.rxpdo, PdoDirection::Rx);
        apply_pdo_map(*backend_, sc.slave_id, sc.txpdo, PdoDirection::Tx);
        // Set modes-of-operation (0x6060, U8) via SDO; it is not mapped cyclically. A drive
        // left in mode 0 never moves. PP=1 / PV=3, from the configured default_mode.
        const std::array<std::byte, 1> mode{static_cast<std::byte>(static_cast<std::uint8_t>(sc.default_mode))};
        backend_->sdo_write(sc.slave_id, kModesOfOp, 0, mode);
    }

    // DC SYNC0 cycle = loop period (A6: must be a 250 us multiple, e.g. 1 ms at 1 kHz).
    const auto cycle_ns = static_cast<std::uint32_t>(kNsPerSec / static_cast<long>(config_.target_loop_rate_hz));

    // Arm SYNC0 in PRE-OP, before config_map_group. The A6 latches its SM sync-type (SM vs DC)
    // at the PRE-OP -> SAFE-OP transition from whether SYNC0 is already armed: arm here and the
    // drive self-selects DC (0x1C32:01 reads 2) and holds OP; arm only after SAFE-OP and it has
    // already chosen SM-sync, then faults Er74.1 "no sync signal" about 1 s into OP. Never write
    // 0x1C32:01 by hand (that force causes AL 0x0030); the PRE-OP arm is the whole trigger. With
    // stock ecx_dcsync0 the 100 ms SyncDelay is covered by config_map, configdc, and the RT loop
    // pumping PD before the first SYNC0 edge.
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
        // the field table (built from config below) carries offsets for the expected map, so a
        // store_le into outputs at a config-derived offset could run past the wire-sized span
        // (out of bounds in the noexcept RT loop). Fail loudly here instead.
        const std::size_t rx_bytes = sc.rxpdo.byte_size();
        const std::size_t tx_bytes = sc.txpdo.byte_size();
        if (info.output_bytes != rx_bytes || info.input_bytes != tx_bytes) {
            throw PdoMappingError("slave " + std::to_string(sc.slave_id) + ": applied RxPDO " + std::to_string(info.output_bytes) +
                                  " B / TxPDO " + std::to_string(info.input_bytes) + " B != configured " + std::to_string(rx_bytes) +
                                  " / " + std::to_string(tx_bytes) + " B (remap did not take)");
        }

        // PdoCache holds the FEEDBACK snapshot (TxPDO/inputs); the command image (RxPDO/outputs)
        // is written directly via outputs(), so the cache is sized to the feedback image only.
        slaves_.emplace_back(sc.slave_id, info.input_bytes);
        SlaveRuntime& rt = slaves_.back();
        rt.io = backend_->slave_io(sc.slave_id);
        rt.rx_fields = build_field_table(sc.slave_id, sc.rxpdo);
        rt.tx_fields = build_field_table(sc.slave_id, sc.txpdo);
    }

    // DC configdc, after config_map_group. Designates the reference clock and writes each
    // slave's system-time offset (0x0920) and propagation delay (0x0928). The SYNC0 arm already
    // happened in PRE-OP above; never write 0x1C32:01 (the drive self-selects DC from the armed SYNC0).
    if (config_.use_distributed_clocks) {
        backend_->configure_dc_configdc();
        // No mlockall here: memory locking is an RT-setup concern, and realtime::setup()
        // already does it (mlockall MCL_CURRENT|MCL_FUTURE) when the Runner's RT thread starts,
        // which is when the SYNC0 PLL first cares (pacing begins after configure() returns at
        // SAFE-OP). Master is thread-free bus policy; memory residency belongs to the RT-setup
        // layer. The CAP_IPC_LOCK / RLIMIT_MEMLOCK guidance lives in realtime::setup's log.
    }

    // Stop at SAFE-OP. SYNC0 is armed (PRE-OP) but its first edge is about 100 ms out (stock
    // SyncDelay), so the brief PRE-OP -> SAFE-OP statecheck (no PD) finishes well before it, and
    // the RT loop is pumping phase-locked PD before the first pulse. The drive latched its DC
    // sync-type at this transition, since SYNC0 was already armed.
    backend_->request_state(0, EcatState::SafeOp);

    // configure() carries no vendor-object knowledge. A vendor fault-reset is consumer policy:
    // consumers run it themselves via the public sdo_write() while they are still the single
    // port owner (before the RT thread spawns).

    // Hand off to the caller's RT loop at SAFE-OP with SYNC0 already armed. It runs
    // bringup_step() -- settle (bounded phase-locked PD), request OP, await OP (hold for OP,
    // Er74.1 cleared, and WKC) -- until operational, so operational_ stays false until then.
    // Reset the FSM.
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
    // The caller owns the cadence (clock_nanosleep + dc_phase_correction on dc_time()); this
    // does the one cyclic exchange and advances the FSM. SYNC0 was already armed in configure()
    // (PRE-OP): settle pumps phase-locked PD a bounded settle, requests OP once, then AwaitOp
    // holds for OP reached, Er74.1 cleared, and WKC holding.
    const int wkc = backend_->exchange();
    last_wkc_.store(wkc, std::memory_order_relaxed);

    switch (bringup_phase_) {
        case BringupPhase::Settle: {
            // Pump phase-locked PD a brief settle so the master's send is disciplined before OP.
            // Do not gate on Er74.1 here: 0x603F=0x8700 in SAFE-OP is the normal pre-sync state,
            // because the A6 completes SYNC0 alignment only once OP cycling starts, so Er74.1
            // clears at OP, not before. Gating on no-Er74.1 here would block the exact transition
            // that works. Non-DC backends need no settle (1 cycle). Then make the initial OP
            // request; AwaitOp keeps re-requesting (reack_op) until it sticks.
            const std::uint32_t target = dc_enabled_ ? (config_.dc_op_gate_cycles == 0 ? 1U : config_.dc_op_gate_cycles) : 1U;
            if (++bringup_settle_count_ >= target) {
                backend_->set_state(0, EcatState::Op);  // writestate only; this loop pumps the transition
                bringup_await_count_ = 0;
                bringup_op_hold_streak_ = 0;
                bringup_phase_ = BringupPhase::AwaitOp;
            }
            return BringupStatus::Gating;
        }
        case BringupPhase::AwaitOp: {
            // Pump the gapless transition while waiting out a slow SAFE-OP -> OP (the A6 takes
            // seconds). Success is a full WKC and no Er74.1 held for op_hold_confirm_cycles:
            // the drive reached OP, SYNC0 aligned (Er74.1 cleared), and PD is exchanging cleanly.
            // Every op_nudge_interval_cycles, run the SAFE-OP recovery (reack_op): ACK a
            // SAFE_OP+ERROR and re-request OP from SAFE_OP. This is self-gating (once the drive
            // is in OP, reack_op is a no-op) and PD keeps flowing, so no watchdog starves. Give
            // up only after the wall-time window (op_await_timeout_ms, converted to cycles in the
            // ctor) without the held-synced state.
            ++bringup_await_count_;
            // Latch the last non-zero AL status code across AWAIT so a give-up can name the
            // cause. Read it before reack_op(0) below: reack ACKs the SAFE_OP+ERROR, momentarily
            // clearing the code, so the value read on the timeout cycle (post-reack) is often 0.
            // This pre-reack read holds the real cause (e.g. 0x0027 "Freerun not supported").
            if (const std::uint16_t al = backend_->al_status_code(1); al != 0) {
                bringup_al_code_ = al;
            }
            if (bringup_await_count_ % op_nudge_interval_cycles_ == 0) {
                backend_->reack_op(0);
            }
            // OP is confirmed by a held full WKC, no sync fault, and plausible drive feedback
            // (drive_present). The last gate matters: the A6 under free-run gives a full WKC
            // while zombie-PDOing (dead statusword), so without drive_present, WKC alone would
            // declare OP on a dead drive and the enable ladder would spin forever. A dead drive
            // keeps drive_present false through the whole window, so the streak never builds,
            // AWAIT times out, and it aborts (and the caller's AL-status diagnostic names AL 0x0027).
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
    last_wkc_.store(wkc, std::memory_order_relaxed);  // raw, every cycle (diagnostic)
    // WKC stats: two relaxed increments per cycle (one conditional). Steady cycles only;
    // bringup_step's exchanges are excluded, because a partial WKC is normal pre-OP and would
    // pollute the bad count. Reset in configure().
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
                // Store the payload (fault_wkc_) first, then publish the flag with a release
                // store so a reader that acquires fault_ == true sees the wkc.
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
    return FieldLocation{it->second.byte_offset, /*present=*/true};
}

FieldLocation Master::tx_field(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const {
    const SlaveRuntime& rt = runtime_for(slave);
    const auto it = rt.tx_fields.find(field_key(index, sub));
    if (it == rt.tx_fields.end()) {
        throw PdoMappingError("slave " + std::to_string(slave) + ": object not in the TxPDO (feedback) map");
    }
    return FieldLocation{it->second.byte_offset, /*present=*/true};
}

FieldLocation Master::resolve_field(const std::map<std::uint32_t, MappedField>& table,
                                    std::uint16_t index,
                                    std::uint8_t sub,
                                    std::size_t want_width,
                                    std::uint16_t slave,
                                    bool is_tx) const {
    const char* const which = is_tx ? "TxPDO (feedback)" : "RxPDO (command)";
    const auto it = table.find(field_key(index, sub));
    if (it == table.end()) {
        // Not-in-map is a map-membership failure, so throw PdoMappingError; the operator fixes
        // it by adding the object to the map. PdoMappingError spans both apply-time
        // (apply_pdo_map) and this runtime access of an un-mapped object. A wrong-width access
        // throws the base Error below, not PdoMappingError, so resolve_rx_optional/
        // resolve_tx_optional do not swallow a wrong-width object as absent.
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
    return FieldLocation{it->second.byte_offset, /*present=*/true};
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
