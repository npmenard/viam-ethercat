#include "ethercat/master.hpp"

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <span>
#include <string>
#include <utility>

#include <sys/mman.h>

#include "ethercat/cia402.hpp"
#include "ethercat/dc_sync.hpp"

namespace ethercat {

namespace {

constexpr int kPrimeCycles = 3;               // exchanges in SAFE-OP so slaves have valid outputs before OP
constexpr std::uint16_t kModesOfOp = 0x6060;  // CiA402 modes-of-operation (U8): PP=1, PV=3; SDO-set in PRE-OP
constexpr long kNsPerSec = 1'000'000'000L;

std::uint32_t field_key(std::uint16_t index, std::uint8_t sub) noexcept {
    return (static_cast<std::uint32_t>(index) << 8U) | sub;
}

}  // namespace

Master::Master(MasterConfig config, std::unique_ptr<EcatBackend> backend) : config_(std::move(config)), backend_(std::move(backend)) {
    if (!backend_) {
        throw ConfigError("Master: null backend");
    }
    if (config_.ifname.empty()) {
        throw ConfigError("Master: empty interface name");
    }
    if (config_.slaves.empty()) {
        throw ConfigError("Master: no slaves configured");
    }
    if (config_.target_loop_rate_hz == 0 || config_.target_loop_rate_hz > 1000) {
        throw ConfigError("Master: target_loop_rate_hz " + std::to_string(config_.target_loop_rate_hz) + " out of range (1..1000)");
    }
}

void Master::init() {
    const std::size_t count = backend_->open(config_.ifname);
    if (count != config_.slaves.size()) {
        throw InitError("EtherCAT bus on '" + config_.ifname + "': found " + std::to_string(count) + " slaves, config expects " +
                        std::to_string(config_.slaves.size()));
    }
}

std::map<std::uint32_t, FieldLocation> Master::build_field_table(std::uint16_t slave, const PdoMap& map) {
    std::map<std::uint32_t, FieldLocation> fields;
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
                fields[field_key(e.index, e.subindex)] = FieldLocation{bit / 8, e.bit_length / 8U};
            }
            bit += e.bit_length;
        }
    }
    return fields;
}

void Master::configure(bool reach_op) {
    // Maps are writable only in PRE-OP and are not stored in EEPROM, so this runs
    // every configure() / power-on.
    backend_->request_state(0, EcatState::PreOp);

    for (const SlaveConfig& sc : config_.slaves) {
        apply_pdo_map(*backend_, sc.slave_id, sc.rxpdo);
        apply_pdo_map(*backend_, sc.slave_id, sc.txpdo);
        // Set modes-of-operation (0x6060, U8) via SDO -- NOT mapped cyclically. A real
        // drive left in mode 0 never moves; this is the one drive-mode write per
        // configure(). PP=1 / PV=3 from the configured default_mode.
        const std::array<std::byte, 1> mode{static_cast<std::byte>(static_cast<std::uint8_t>(sc.default_mode))};
        backend_->sdo_write(sc.slave_id, kModesOfOp, 0, mode);
    }

    backend_->map_process_data();
    expected_wkc_ = backend_->expected_wkc();

    slaves_.clear();
    for (const SlaveConfig& sc : config_.slaves) {
        const SlaveInfo info = backend_->slave_info(sc.slave_id);

        // Validate the APPLIED (wire) image against the CONFIGURED map. If a real
        // drive silently rejected part of the remap, map_process_data lays out the
        // drive's default image while the field table (built from config below)
        // carries offsets for the expected map -- a store_le into outputs at a
        // config-derived offset could then run past the wire-sized span (OOB in
        // the noexcept RT loop). Fail loudly here instead.
        const std::size_t rx_bytes = sc.rxpdo.byte_size();
        const std::size_t tx_bytes = sc.txpdo.byte_size();
        if (info.output_bytes != rx_bytes || info.input_bytes != tx_bytes) {
            throw PdoMappingError("slave " + std::to_string(sc.slave_id) + ": applied RxPDO " + std::to_string(info.output_bytes) +
                                  " B / TxPDO " + std::to_string(info.input_bytes) + " B != configured " + std::to_string(rx_bytes) +
                                  " / " + std::to_string(tx_bytes) + " B (remap did not take)");
        }

        // PdoCache(rx = FEEDBACK size (TxPDO/inputs), tx = COMMAND size (RxPDO/outputs)).
        slaves_.emplace_back(sc.slave_id, info.input_bytes, info.output_bytes);
        SlaveRuntime& rt = slaves_.back();
        rt.io = backend_->slave_io(sc.slave_id);
        rt.rx_fields = build_field_table(sc.slave_id, sc.rxpdo);
        rt.tx_fields = build_field_table(sc.slave_id, sc.txpdo);
    }

    backend_->request_state(0, EcatState::SafeOp);
    // Distributed Clocks: configure SYNC0 at SAFE-OP before OP. Required by
    // drives that support only DC sync (the A6-EC faults out of OP -- Er74.1 "no
    // sync signal", WKC->0 -- without it). cycle = loop period; the A6 needs an
    // integer multiple of 250 us (1 kHz -> 1 ms is valid).
    if (config_.use_distributed_clocks) {
        const auto cycle_ns = static_cast<std::uint32_t>(kNsPerSec / static_cast<long>(config_.target_loop_rate_hz));
        backend_->configure_dc_sync(cycle_ns);
        // Lock CURRENT memory (the IOmap + SOEM context are already resident after
        // map_process_data) right before the warmup. The warmup is alloc-free
        // (send/receive over the pre-allocated context), so MCL_CURRENT covers its
        // whole working set -- a page fault mid-warmup is a ms spike that spoils the
        // SYNC0 PLL lock. We deliberately do NOT use MCL_FUTURE here: this can run on
        // a non-RT thread that later spawns the RT jthread, and MCL_FUTURE would make
        // that thread's stack alloc hit RLIMIT_MEMLOCK -> EAGAIN. Best-effort but
        // NEVER silent (a silent fail re-introduces the spike it prevents).
        if (mlockall(MCL_CURRENT) != 0) {
            (void)std::fprintf(stderr,
                               "[ethercat] mlockall(MCL_CURRENT) failed (errno=%d) before the DC warmup: grant "
                               "CAP_IPC_LOCK / RLIMIT_MEMLOCK=infinity; the SYNC0 PLL lock may be unreliable.\n",
                               errno);
        }
        // Run a PHASE-LOCKING warmup before OP: pace exchanges at the SYNC0 cycle
        // AND run the DC phase-lock PI each cycle, so we don't just prime the PLL but
        // converge our send phase into the SYNC0 window BEFORE requesting OP -- then
        // the bus enters OP already aligned (WKC 3/3 from cycle 0) instead of dropping
        // WKC while the post-OP loop is still pulling the phase in. Break early once
        // locked for a few consecutive cycles. dc_lock_cycles = 0 skips it (sim).
        // PROVISIONAL (architect, runbook §5.2): this runs on the caller's (possibly
        // non-RT) thread; memory is locked above so no page-fault spikes, but
        // SCHED_OTHER jitter remains -- if the bench shows it can't hold lock, move
        // the warmup + OP transition into the RT thread prelude (design "(b)").
        // Lock the phase at this DC-time offset. -1 = auto = mid-cycle (cycle/2):
        // off the SYNC0 edge so jitter never crosses the pulse.
        const std::int64_t shift = config_.dc_sync_shift_ns < 0 ? static_cast<std::int64_t>(cycle_ns) / 2 : config_.dc_sync_shift_ns;
        timespec next{};
        (void)clock_gettime(CLOCK_MONOTONIC, &next);
        std::int64_t dc_integral = 0;
        int locked_streak = 0;
        for (std::uint32_t i = 0; reach_op && i < config_.dc_lock_cycles; ++i) {
            (void)backend_->exchange();
            const std::int64_t dct = backend_->dc_time();
            const long corr = dc_phase_correction(dct, static_cast<std::int64_t>(cycle_ns), dc_integral, shift);
            locked_streak = dc_phase_locked(dct, static_cast<std::int64_t>(cycle_ns), shift) ? locked_streak + 1 : 0;
            if (locked_streak >= 50) {
                break;  // phase held in-band for 50 cycles -> enter OP locked
            }
            next.tv_nsec += static_cast<long>(cycle_ns) + corr;
            while (next.tv_nsec >= kNsPerSec) {
                next.tv_nsec -= kNsPerSec;
                next.tv_sec += 1;
            }
            while (next.tv_nsec < 0) {
                next.tv_nsec += kNsPerSec;
                next.tv_sec -= 1;
            }
            (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
        }
    } else if (reach_op) {
        for (int i = 0; i < kPrimeCycles; ++i) {
            (void)backend_->exchange();
        }
    }

    fault_.store(false, std::memory_order_relaxed);
    consecutive_wkc_errors_ = 0;
    if (reach_op) {
        backend_->request_state(0, EcatState::Op);
        if (backend_->slave_state(0) != EcatState::Op) {
            throw InitError("EtherCAT bus on '" + config_.ifname + "': not all slaves reached OPERATIONAL (bus is " +
                            to_string(backend_->slave_state(0)) + ")");
        }
        // Post-OP DC settle grace (no WKC latch while the phase finishes locking).
        settle_remaining_ = config_.use_distributed_clocks ? config_.dc_settle_cycles : 0;
        operational_.store(true, std::memory_order_relaxed);
    } else {
        // CALLER-DRIVEN bring-up: leave the bus in SAFE-OP with DC enabled; the
        // caller's CONTINUOUS cyclic loop runs the phase-lock warmup, requests OP
        // (request_op()) once locked, and keeps cycling -- so a DC drive sees an
        // unbroken stream of frames through SAFE-OP->OP (no gap -> no Er74). A big
        // grace covers the expected short WKC across that whole window.
        settle_remaining_ = config_.dc_lock_cycles + config_.dc_settle_cycles + static_cast<std::uint32_t>(kPrimeCycles);
        operational_.store(true, std::memory_order_relaxed);
    }
}

void Master::request_op() noexcept {
    backend_->set_state(0, EcatState::Op);  // writestate only; the caller's loop pumps the transition
}

void Master::process() noexcept {
    const int wkc = backend_->exchange();
    last_wkc_.store(wkc, std::memory_order_relaxed);  // raw, every cycle (diagnostic)
    // Post-OP DC settle grace: while it lasts, fully clear the latch state every
    // cycle, so NOTHING (not even a bad streak during the grace) can carry past the
    // grace boundary and trip a spurious latch the instant it ends. The DC phase PI
    // is still pulling into the window; a few partial-processing cycles are benign.
    const bool in_grace = settle_remaining_ > 0;
    if (in_grace) {
        --settle_remaining_;
        consecutive_wkc_errors_ = 0;
    }
    if (wkc < 0 || wkc < expected_wkc_) {
        if (!in_grace) {
            ++consecutive_wkc_errors_;
            if (consecutive_wkc_errors_ >= config_.max_consecutive_wkc_errors) {
                // Store the payload (fault_wkc_) first, then publish the flag with a
                // release store so a reader that acquires fault_==true sees the wkc.
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
    throw ConfigError("Master: unknown slave " + std::to_string(slave));
}

const Master::SlaveRuntime& Master::runtime_for(std::uint16_t slave) const {
    for (const SlaveRuntime& rt : slaves_) {
        if (rt.slave_id == slave) {
            return rt;
        }
    }
    throw ConfigError("Master: unknown slave " + std::to_string(slave));
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
    return it->second;
}

FieldLocation Master::tx_field(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const {
    const SlaveRuntime& rt = runtime_for(slave);
    const auto it = rt.tx_fields.find(field_key(index, sub));
    if (it == rt.tx_fields.end()) {
        throw PdoMappingError("slave " + std::to_string(slave) + ": object not in the TxPDO (feedback) map");
    }
    return it->second;
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
