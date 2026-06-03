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
constexpr int kDcPreopLockStreak = 200;                // PRE-OP: hold phase-lock this many cycles before crossing to SAFE-OP
constexpr std::uint32_t kDcSafeopMeasureCycles = 500;  // SAFE-OP: keep pumping (gapless) while the drive measures its cycle
constexpr int kDcWarmupLockStreak = 50;                // post-SAFE-OP warmup: streak before requesting OP

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

    // Apply a list of driver SDO writes. A write marked optional that the drive
    // rejects (read-only object / CoE abort) is logged and skipped, not fatal --
    // for diagnostic / best-effort tuning writes; a mandatory write still throws.
    const auto apply_sdo_writes = [this](std::uint16_t slave_id, const std::vector<SdoWrite>& writes) {
        for (const SdoWrite& w : writes) {
            if (w.optional) {
                try {
                    backend_->sdo_write(slave_id, w.index, w.subindex, w.data);
                } catch (const Error& e) {
                    (void)std::fprintf(stderr,
                                       "[ethercat] optional SDO write to slave %u object 0x%04X:%02X rejected (continuing): %s\n",
                                       static_cast<unsigned>(slave_id),
                                       static_cast<unsigned>(w.index),
                                       static_cast<unsigned>(w.subindex),
                                       e.what());
                }
            } else {
                backend_->sdo_write(slave_id, w.index, w.subindex, w.data);
            }
        }
    };

    for (const SlaveConfig& sc : config_.slaves) {
        // PRE-OP SDO writes BEFORE the remap: drive-tuning params that must land while
        // the drive is quiescent and the SM mapping is still default (config DATA; no
        // drive specifics here). A bad object/length surfaces as the backend's
        // PdoMappingError carrying the CoE abort code.
        apply_sdo_writes(sc.slave_id, sc.preop_sdo_writes);
        apply_pdo_map(*backend_, sc.slave_id, sc.rxpdo);
        apply_pdo_map(*backend_, sc.slave_id, sc.txpdo);
        // AFTER the PDO assignment: SM-synchronization writes (0x1C32:01/0x1C33:01 sync
        // type). MUST follow the 0x1C12/0x1C13 assignment -- several drives re-default
        // 0x1C32 when the assignment changes, so a pre-assignment sync-type write is
        // clobbered (ETG startup order: map -> assign -> SM-sync).
        apply_sdo_writes(sc.slave_id, sc.postremap_sdo_writes);
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

    // Distributed Clocks: the SOEM-author (Arthur Ketels) canonical order. A DC drive
    // proves it is in sync from synchronized, DC-phase-locked PDO TRAFFIC observed in
    // SAFE-OP; only then will it permit OP. So the sequence is:
    //   configdc (PRE-OP, offsets only) -> request SAFE-OP -> dcsync0 (arm SYNC0 on a
    //   fresh live clock) -> pump a phase-locked PD loop in SAFE-OP -> request OP.
    // Arming SYNC0 in PRE-OP (the old order) schedules it off a not-yet-disciplined
    // clock and the drive never sees the synchronized transfer it requires -> Er74 /
    // AL 0x0030. cycle = loop period; the A6 needs a multiple of 250 us (1 ms valid).
    const auto cycle_ns = static_cast<std::uint32_t>(kNsPerSec / static_cast<long>(config_.target_loop_rate_hz));

    // DC step 1 (PRE-OP): configdc -- reference clock + system-time offset + delay.
    if (config_.use_distributed_clocks) {
        backend_->configure_dc_configdc();
    }

    const std::int64_t dc_shift = config_.dc_sync_shift_ns < 0 ? static_cast<std::int64_t>(cycle_ns) / 2 : config_.dc_sync_shift_ns;
    if (config_.use_distributed_clocks) {
        // Lock CURRENT memory (the IOmap + SOEM context are resident after
        // map_process_data) before ANY RT-paced pumping (the PRE-OP lock, the measure
        // window, the warmup), so a page fault never spikes the phase. NOT MCL_FUTURE:
        // this can run on a non-RT thread that later spawns the RT jthread, and
        // MCL_FUTURE would make that thread's stack alloc hit RLIMIT_MEMLOCK -> EAGAIN.
        if (mlockall(MCL_CURRENT) != 0) {
            (void)std::fprintf(stderr,
                               "[ethercat] mlockall(MCL_CURRENT) failed (errno=%d) before the DC warmup: grant "
                               "CAP_IPC_LOCK / RLIMIT_MEMLOCK=infinity; the SYNC0 PLL lock may be unreliable.\n",
                               errno);
        }

        // DC step 1.5 -- PHASE-LOCK THE MASTER IN PRE-OP, BEFORE crossing to SAFE-OP. The
        // A6 latches its RO SM cycle (0x1C32:02) from the FIRST SM2-event period it sees
        // at the PRE-OP->SAFE-OP transition, then validates THAT at OP -- it does NOT
        // re-derive it from SYNC0. If the master is still phase-locking when SM2 turns on,
        // that first period is the jittery ~999 us unlocked transient -> latched ->
        // Er74.0 (0x6320) cycle error at OP. ec_DCtime updates via the FRMW datagram even
        // in PRE-OP, so the master CAN lock here. Lock FIRST, then cross over so the
        // drive's first measurement is the clean, locked 1 ms rate. (bench: team-lead.)
        // dc_lock_cycles == 0 (sim / module) skips the warmup entirely -- request SAFE-OP
        // straight away (SimBackend has no real DC clock to lock to).
        if (config_.dc_lock_cycles > 0) {
            timespec next{};
            (void)clock_gettime(CLOCK_MONOTONIC, &next);
            std::int64_t dc_integral = 0;
            const int preop_streak = phase_lock_pump(cycle_ns, dc_shift, config_.dc_lock_cycles, kDcPreopLockStreak, next, dc_integral);
            (void)std::fprintf(stderr,
                               "[ethercat] PRE-OP phase-lock: streak=%d (target %d) before SAFE-OP%s\n",
                               preop_streak,
                               kDcPreopLockStreak,
                               preop_streak >= kDcPreopLockStreak ? " -> LOCKED" : " -> NOT locked (cap hit)");

            // Cross into SAFE-OP with the master ALREADY locked, then keep pumping
            // through the drive's cycle-measurement window so the value it latches into
            // 0x1C32:02 is the clean locked 1 ms.
            backend_->request_state(0, EcatState::SafeOp);
            // RE-BASE the deadline: request_state() blocks on the AL statecheck WITHOUT
            // pumping process data, so `next` is now several ms in the PAST. Without this
            // the measure window's first cycles fire back-to-back catching up -> a sub-ms
            // FIRST SM2-event period -> exactly the mis-measurement we're preventing (the
            // A6 latches that first period into 0x1C32:02). Fresh base = the first
            // post-SAFE-OP frame lands one clean cycle from now. The integral (phase) is
            // preserved across the re-base, so the master is still locked.
            (void)clock_gettime(CLOCK_MONOTONIC, &next);
            (void)phase_lock_pump(cycle_ns, dc_shift, kDcSafeopMeasureCycles, 0, next, dc_integral);
        } else {
            backend_->request_state(0, EcatState::SafeOp);
        }

        // DC step 2: arm SYNC0 -- ONLY on the self-contained path (reach_op=true, e.g. the
        // module). On the caller-driven path (reach_op=false, e.g. a6_validate) the arm is
        // DEFERRED to the caller's RT loop via Master::arm_dc_sync(), which arms a few
        // cycles in once it has re-confirmed phase-lock.
        if (reach_op) {
            backend_->configure_dc_sync(cycle_ns, config_.dc_sync0_shift_ns, config_.dc_sync_start_delay_ns);
        }
    } else {
        backend_->request_state(0, EcatState::SafeOp);
    }

    // POST-DC SDO writes -- the ETG.1020 cycle-time handshake (0x1C32:0a Sync0 cycle +
    // :08 Get-Cycle) that populates the read-only 0x1C32:02 the drive validates. Needs
    // the ESC cycle register 0x09A0 live, i.e. AFTER the SYNC0 arm. (0x1C32:01 = DC-mode
    // switch stays in postremap, PRE-OP -- writable only before configdc.) Applied here
    // ONLY on the self-contained path (reach_op=true): the arm has already run above. On
    // the caller-driven path (reach_op=false) the arm is deferred to the caller's RT
    // loop, so the caller applies these post-arm via Master::apply_postdc_writes().
    if (reach_op) {
        for (const SlaveConfig& sc : config_.slaves) {
            apply_sdo_writes(sc.slave_id, sc.postdc_sdo_writes);
        }
    }

    // POST-DC SETTLE: pump paced PD so the drive APPLIES the DC config -- copies the
    // live ESC SYNC0 cycle (0x09A0) into the read-only CoE 0x1C32:02. Optionally poll a
    // CoE object each cycle and break early once it reads non-zero (config applied).
    if (reach_op && config_.use_distributed_clocks && config_.dc_postwrite_settle_cycles > 0) {
        const std::uint16_t poll_slave = config_.slaves.front().slave_id;
        timespec next{};
        (void)clock_gettime(CLOCK_MONOTONIC, &next);
        std::array<std::byte, 4> poll_buf{};
        bool applied = false;
        std::uint32_t i = 0;
        for (; i < config_.dc_postwrite_settle_cycles && !applied; ++i) {
            (void)backend_->exchange();
            if (config_.dc_settle_poll_index != 0) {
                try {
                    const std::size_t n =
                        backend_->sdo_read(poll_slave, config_.dc_settle_poll_index, config_.dc_settle_poll_sub, poll_buf);
                    std::uint32_t v = 0;
                    for (std::size_t b = 0; b < n && b < poll_buf.size(); ++b) {
                        v |= static_cast<std::uint32_t>(std::to_integer<std::uint8_t>(poll_buf[b])) << (8U * b);
                    }
                    applied = (v != 0);
                } catch (const Error&) {  // NOLINT(bugprone-empty-catch) -- object not readable yet; settle another cycle
                }
            }
            next.tv_nsec += static_cast<long>(cycle_ns);
            while (next.tv_nsec >= kNsPerSec) {
                next.tv_nsec -= kNsPerSec;
                next.tv_sec += 1;
            }
            (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
        }
        if (config_.dc_settle_poll_index != 0) {
            (void)std::fprintf(stderr,
                               "[ethercat] post-DC settle: %u cycles, poll 0x%04X:%02X %s\n",
                               i,
                               static_cast<unsigned>(config_.dc_settle_poll_index),
                               static_cast<unsigned>(config_.dc_settle_poll_sub),
                               applied ? "-> NON-ZERO (DC config applied)" : "-> still 0 after settle (NOT applied)");
        }
    }

    if (config_.use_distributed_clocks && reach_op) {
        // Post-SAFE-OP, SYNC0-armed warmup before OP (self-contained path): re-confirm
        // the phase-lock (the postdc settle above may have broken the cadence), so the
        // bus enters OP already aligned (WKC 3/3 from cycle 0). The PRE-OP lock + measure
        // window already ran; this is the final convergence before the OP request.
        // dc_lock_cycles = 0 skips it (sim). a6_validate (reach_op=false) instead owns the
        // whole arm->lock->gate->OP sequence in its own continuous loop.
        timespec next{};
        (void)clock_gettime(CLOCK_MONOTONIC, &next);
        std::int64_t dc_integral = 0;
        (void)phase_lock_pump(cycle_ns, dc_shift, config_.dc_lock_cycles, kDcWarmupLockStreak, next, dc_integral);
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

int Master::phase_lock_pump(std::uint32_t cycle_ns,
                            std::int64_t shift_ns,
                            std::uint32_t max_cycles,
                            int target_streak,
                            timespec& next,
                            std::int64_t& integral) noexcept {
    int streak = 0;
    for (std::uint32_t i = 0; i < max_cycles; ++i) {
        (void)backend_->exchange();
        const std::int64_t dct = backend_->dc_time();
        const long corr = dc_phase_correction(dct, static_cast<std::int64_t>(cycle_ns), integral, shift_ns);
        streak = dc_phase_locked(dct, static_cast<std::int64_t>(cycle_ns), shift_ns) ? streak + 1 : 0;
        if (target_streak > 0 && streak >= target_streak) {
            return streak;  // phase held in-band -> caller may advance state
        }
        next.tv_nsec += static_cast<long>(cycle_ns) + corr;
        while (next.tv_nsec >= kNsPerSec) {
            next.tv_nsec -= kNsPerSec;
            next.tv_sec += 1;
        }
        while (next.tv_nsec < 0) {  // a correction can push the deadline slightly negative
            next.tv_nsec += kNsPerSec;
            next.tv_sec -= 1;
        }
        (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
    return streak;
}

void Master::arm_dc_sync() noexcept {
    const auto cycle_ns = static_cast<std::uint32_t>(kNsPerSec / static_cast<long>(config_.target_loop_rate_hz));
    try {
        backend_->arm_dc_sync(cycle_ns, config_.dc_sync0_shift_ns, config_.dc_sync_start_delay_ns);  // short start delay, no prime
    } catch (const std::exception& e) {
        (void)std::fprintf(stderr, "[ethercat] arm_dc_sync failed: %s\n", e.what());
    }
}

void Master::apply_postdc_writes() noexcept {
    for (const SlaveConfig& sc : config_.slaves) {
        for (const SdoWrite& w : sc.postdc_sdo_writes) {
            try {
                backend_->sdo_write(sc.slave_id, w.index, w.subindex, w.data);
            } catch (const std::exception& e) {
                (void)std::fprintf(stderr,
                                   "[ethercat] post-arm SDO write to slave %u object 0x%04X:%02X %s (continuing): %s\n",
                                   static_cast<unsigned>(sc.slave_id),
                                   static_cast<unsigned>(w.index),
                                   static_cast<unsigned>(w.subindex),
                                   w.optional ? "rejected" : "FAILED",
                                   e.what());
            }
        }
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
