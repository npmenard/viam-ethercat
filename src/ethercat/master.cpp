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
// The AWAIT_OP bounds (hold-confirm / nudge-interval / give-up) moved into MasterConfig
// (#42): they carry A6/ec_sample-derived magnitudes + a wall-time meaning that depends on
// the loop rate, so they are config (with the same defaults), derived once in the ctor.

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
    // Derive the AWAIT_OP bounds ONCE from config (#42) -- no per-cycle math/clamping on
    // the bring-up path. The counts clamp 0 -> 1 (a zero hold-confirm would declare OP on
    // no held evidence; a zero nudge interval would divide by zero). The give-up bound is
    // configured in WALL TIME and converted at the configured rate, so the ec_sample
    // patience window is rate-independent (e.g. 30 s at 250 Hz = 7'500 cycles, not the
    // 2 minutes the old fixed 30'000-cycle constant would have meant).
    op_hold_confirm_cycles_ = std::max(config_.op_hold_confirm_cycles, 1U);
    op_nudge_interval_cycles_ = std::max(config_.op_nudge_interval_cycles, 1U);
    const std::uint64_t await_cycles = (static_cast<std::uint64_t>(config_.op_await_timeout_ms) * config_.target_loop_rate_hz) / 1000ULL;
    op_await_bound_cycles_ = static_cast<std::uint32_t>(std::clamp<std::uint64_t>(await_cycles, 1, UINT32_MAX));

    // #44: validate the loop rate against each slave's declared SYNC0 cycle granularity
    // UP FRONT -- a non-multiple cycle otherwise surfaces only as a cryptic fault AT OP
    // ENTRY (the A6 Er74.0 "cycle error"), the worst place to debug a config mistake.
    // Uses the SAME truncated-cycle arithmetic configure() arms SYNC0 with, so the value
    // checked is the value the drive sees. Granularity is per-slave config DATA (0 = no
    // constraint); only meaningful when DC/SYNC0 is in play.
    if (config_.use_distributed_clocks) {
        const std::uint64_t cycle_ns = static_cast<std::uint64_t>(kNsPerSec) / config_.target_loop_rate_hz;
        for (const SlaveConfig& sc : config_.slaves) {
            const std::uint32_t g = sc.sync_cycle_granularity_ns;
            if (g == 0 || cycle_ns % g == 0) {
                continue;
            }
            // Suggest the nearest rates (within the validated 1..1000 Hz range) whose
            // truncated cycle IS a multiple -- scanned with the same arithmetic as the check.
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
            throw ConfigError("Master: slave " + std::to_string(sc.slave_id) + " declares sync_cycle_granularity_ns=" + std::to_string(g) +
                              " but target_loop_rate_hz=" + std::to_string(config_.target_loop_rate_hz) + " gives a " +
                              std::to_string(cycle_ns) +
                              " ns SYNC0 cycle that is not a multiple -- the drive would reject it at OP entry. Nearest valid"
                              " rates:" +
                              (nearest.empty() ? " none in 1..1000 Hz" : nearest));
        }
    }
}

void Master::init() {
    const std::size_t count = backend_->open(config_.ifname);
    if (count != config_.slaves.size()) {
        throw InitError("EtherCAT bus on '" + config_.ifname + "': found " + std::to_string(count) + " slaves, config expects " +
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
        // The STRUCTURAL PDO remap (#TODO-2 keeps this in Master -- it's intrinsic to the
        // init->map sequence, not consumer policy): assign 0x1600/0x1A00 to SM2/SM3
        // (0x1C12/0x1C13) + write the entry lists. The generic preop/postremap SDO lists
        // that used to bracket this are GONE -- setup-SDO policy is the consumer's, run
        // via Master::sdo_write() post-configure while it is the single port owner.
        //
        // ETG ORDERING LESSON (preserve for a future named SM-sync hook, #TODO-2 / DA):
        // an SM-sync-type write (0x1C32:01 / 0x1C33:01) MUST go AFTER this apply_pdo_map,
        // not before -- several drives RE-DEFAULT 0x1C32 when the PDO assignment changes,
        // so a sync-type write done before the assignment is silently clobbered. (The A6
        // needs none -- it self-selects DC from the PRE-OP SYNC0 arm, #20 -- but the next
        // drive that needs an explicit sync-type write must place its hook here, post-remap.)
        apply_pdo_map(*backend_, sc.slave_id, sc.rxpdo, PdoDirection::Rx);
        apply_pdo_map(*backend_, sc.slave_id, sc.txpdo, PdoDirection::Tx);
        // Set modes-of-operation (0x6060, U8) via SDO -- NOT mapped cyclically. A real
        // drive left in mode 0 never moves; this is the one drive-mode write per
        // configure(). PP=1 / PV=3 from the configured default_mode.
        const std::array<std::byte, 1> mode{static_cast<std::byte>(static_cast<std::uint8_t>(sc.default_mode))};
        backend_->sdo_write(sc.slave_id, kModesOfOp, 0, mode);
    }

    // DC SYNC0 cycle = loop period (A6: must be a 250 us multiple, e.g. 1 ms @ 1 kHz).
    const auto cycle_ns = static_cast<std::uint32_t>(kNsPerSec / static_cast<long>(config_.target_loop_rate_hz));

    // ARM SYNC0 IN PRE-OP, BEFORE config_map_group -- ec_sample's exact order (bench /
    // CLAUDE.md). The A6 latches its SM sync-type (SM vs DC) at the PRE-OP->SAFE-OP
    // transition from whether SYNC0 is ALREADY armed: arm here and the drive self-selects
    // DC (0x1C32:01 reads 2) and holds OP; arm only after SAFE-OP and it has already chosen
    // SM-sync -> Er74.1 "no sync signal" ~1s into OP. We still NEVER write 0x1C32:01 (that
    // force was the self-inflicted AL 0x0030); the PRE-OP arm is the whole trigger. Stock
    // ecx_dcsync0 -- the 100 ms SyncDelay is covered by config_map+configdc + the RT loop
    // pumping PD before the first SYNC0 edge.
    if (config_.use_distributed_clocks) {
        backend_->arm_dc_sync(cycle_ns, config_.dc_sync0_shift_ns);
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

    // DC configdc -- AFTER config_map_group (ec_sample order: dcsync0 -> map -> configdc).
    // Designates the reference clock + writes each slave's system-time offset (0x0920) +
    // propagation delay (0x0928). The SYNC0 arm already happened in PRE-OP above; we NEVER
    // write 0x1C32:01 (the drive self-selects DC from the armed SYNC0).
    if (config_.use_distributed_clocks) {
        backend_->configure_dc_configdc();
        // NOTE (#TODO-6): no mlockall here. Memory locking is an RT-SETUP concern, and
        // realtime::setup() already does it (mlockall MCL_CURRENT|MCL_FUTURE) when the
        // Runner's RT thread starts -- which is the only point the SYNC0 PLL cares about
        // (pacing begins post-start, after configure() returns at SAFE-OP). A second
        // mlockall here was redundant AND a layering leak: Master is thread-free bus
        // policy (#31/#47); memory residency belongs to the RT-setup layer. The
        // CAP_IPC_LOCK / RLIMIT_MEMLOCK errno guidance lives in realtime::setup's log.
    }

    // Stop at SAFE-OP. SYNC0 is armed (PRE-OP) but its first edge is ~100 ms out (stock
    // SyncDelay), so the brief PRE-OP->SAFE-OP statecheck (no PD) finishes well before it
    // -- the RT loop is pumping phase-locked PD before the first pulse. The drive latched
    // DC sync-type at this transition (SYNC0 already armed).
    backend_->request_state(0, EcatState::SafeOp);

    // NOTE (#39): configure() carries ZERO vendor-object knowledge. The vendor fault-reset
    // that used to fire here is CONSUMER policy now -- consumers run it themselves via the
    // public sdo_write() while they are still the single port owner (pre-RT-spawn).

    // Hand off to the caller's RT loop at SAFE-OP with SYNC0 already armed. It runs
    // bringup_step() -- SETTLE (bounded phase-locked PD) -> request OP -> AWAIT_OP (hold
    // for OP + Er74.1-cleared + WKC) -- until OPERATIONAL, so operational_ stays false
    // until then. Reset the FSM.
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
    // The caller owns the cadence (clock_nanosleep + dc_phase_correction on dc_time());
    // this does the one cyclic exchange + advances the FSM. SYNC0 was already armed in
    // configure() (PRE-OP, per ec_sample): SETTLE pumps phase-locked PD a bounded settle,
    // requests OP ONCE, then AWAIT_OP holds for "OP reached + Er74.1 cleared + WKC holds".
    const int wkc = backend_->exchange();
    last_wkc_.store(wkc, std::memory_order_relaxed);

    switch (bringup_phase_) {
        case BringupPhase::Settle: {
            // Pump phase-locked PD a brief settle so the master's send is disciplined
            // before OP (ec_sample settles RT PD ~400 ms before requesting OP). We do NOT
            // gate on Er74.1 here: 0x603F=0x8700 in SAFE-OP is the NORMAL pre-sync state --
            // the A6 completes SYNC0 alignment only once OP cycling starts, so Er74.1
            // clears AT OP, not before (ec_sample requests OP with it present and succeeds).
            // Gating on no-Er74.1 here would block the exact transition that works. Non-DC
            // backends need no settle (1 cycle). Then make the INITIAL OP request; AwaitOp keeps
            // re-requesting (reack_op) until it sticks, the way ec_sample waits out the A6.
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
            // Pump the gapless transition while WAITING OUT a slow SAFE-OP->OP (the A6 takes
            // seconds). Success = full WKC AND no Er74.1 held op_hold_confirm_cycles -- i.e. the
            // drive reached OP, SYNC0 aligned (Er74.1 cleared), and PD is exchanging cleanly.
            // Every op_nudge_interval_cycles, run ec_sample's SAFE-OP recovery (reack_op): ACK a
            // SAFE_OP+ERROR / re-request OP from SAFE_OP. This is self-gating -- once the drive is
            // in OP, reack_op is a no-op -- and PD keeps flowing (no watchdog starve). Only after
            // the generous wall-time give-up window (op_await_timeout_ms, converted to cycles in
            // the ctor) without the held-synced state do we give up. Bounds are MasterConfig (#42).
            ++bringup_await_count_;
            // #71/#25: latch the last NON-ZERO AL status code across AWAIT so a give-up can name the
            // cause. Read it BEFORE reack_op(0) below -- reack ACKs the SAFE_OP+ERROR, momentarily
            // clearing the code, so the value read on the timeout cycle (post-reack) is often 0. This
            // pre-reack read holds the real cause (e.g. 0x0027 "Freerun not supported").
            if (const std::uint16_t al = backend_->al_status_code(1); al != 0) {
                bringup_al_code_ = al;
            }
            if (bringup_await_count_ % op_nudge_interval_cycles_ == 0) {
                backend_->reack_op(0);
            }
            // #71/#25: OP is confirmed by a HELD full WKC AND no sync fault AND plausible drive
            // feedback (drive_present). The last gate is load-bearing: the A6 under free-run gives a
            // FULL WKC while zombie-PDOing (dead statusword) -- without drive_present, WKC alone would
            // declare OP on a dead drive and the enable ladder would spin forever. A dead drive keeps
            // drive_present false through the whole window -> the streak never builds -> AWAIT times
            // out -> Aborted (and the caller's AL-status diagnostic names AL 0x0027).
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
    // Drain any Tpdo-submitted frame into the live command image BEFORE the exchange, so a
    // submit() lands on the very next process(). take_outputs is the RT-side lock-free take
    // (this runs on the RT thread); it returns 0 and leaves the image untouched when nothing
    // is staged -- so the direct-write path (servo module / a6_validate writing outputs()
    // directly) is unaffected (#30 §3: "submit() -> transmitted next cycle, via TxStaging").
    for (SlaveRuntime& rt : slaves_) {
        (void)rt.cache.take_outputs(rt.io.outputs);
    }
    const int wkc = backend_->exchange();
    last_wkc_.store(wkc, std::memory_order_relaxed);  // raw, every cycle (diagnostic)
    // WKC stats (#40 item 4): two relaxed increments per cycle (one conditional). Steady
    // cycles only -- bringup_step's exchanges are deliberately excluded (partial WKC is
    // NORMAL pre-OP and would pollute the bad count). Reset in configure().
    total_cycles_.fetch_add(1, std::memory_order_relaxed);
    if (wkc < 0 || wkc < expected_wkc_) {
        bad_cycles_.fetch_add(1, std::memory_order_relaxed);
    }
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
        // NOT-IN-MAP is a MAP-MEMBERSHIP failure -> PdoMappingError ("the map can't satisfy you";
        // distinct operator fix = "add it to the map"). PdoMappingError spans BOTH apply-time
        // (apply_pdo_map) and this runtime access of an un-mapped object. A MALFORMED access
        // (wrong width / past frame) is the separate PdoAccessError tier below.
        throw PdoMappingError("slave " + std::to_string(slave) + ": object " + std::to_string(index) + ":" + std::to_string(sub) +
                              " is not in the " + which + " map");
    }
    // #30 §5 width assertion (PROVISIONAL, pending the user's call via team-lead -- built but
    // trivially removable): the Field's T must match the mapped object's width. Catches a silent
    // wrong-width access (e.g. an int16 alias against a 32-bit-mapped object would read 2 of 4
    // bytes in-bounds, no throw). Checked against the INTERNAL table's bit_length (the public
    // FieldLocation is offset-only post-P2c); build_field_table guarantees bit_length % 8 == 0.
    const std::size_t mapped_width = it->second.bit_length / 8U;
    if (mapped_width != want_width) {
        throw PdoAccessError("slave " + std::to_string(slave) + ": object " + std::to_string(index) + ":" + std::to_string(sub) +
                             " width mismatch -- the Field type is " + std::to_string(want_width) + " byte(s) but the object is mapped " +
                             std::to_string(mapped_width) + " byte(s)");
    }
    return FieldLocation{it->second.byte_offset, /*present=*/true};
}

Rpdo Master::read_rpdo(std::uint16_t slave) const {
    (void)runtime_for(slave);                      // validate the slave id (throws ConfigError, clear text)
    return Rpdo(read_inputs(slave), this, slave);  // ONE seqlock read, copied into the frame-consistent snapshot
}

Tpdo Master::make_tpdo(std::uint16_t slave) {
    SlaveRuntime& rt = runtime_for(slave);               // validate + get the cache (throws ConfigError, clear text)
    return Tpdo(rt.io.outputs, this, &rt.cache, slave);  // seed from the CURRENT command image
}

void Master::sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) {
    // Port-ownership guard (#39): a blocking mailbox transfer concurrent with a running
    // RT loop starves cyclic LRW (the ec_sample 0x001B lesson). rt_active_ is the
    // consumer-DECLARED RT phase; while set, refuse loudly instead of corrupting timing.
    if (rt_active_.load(std::memory_order_acquire)) {
        throw ConfigError("Master::sdo_write: refused during a declared RT phase (slave " + std::to_string(slave) + " object " +
                          std::to_string(index) + ":" + std::to_string(sub) +
                          ") -- SDO is pre-RT-spawn/post-RT-join only; steady-state access is the #22 queue");
    }
    backend_->sdo_write(slave, index, sub, data);
}

std::size_t Master::sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) {
    if (rt_active_.load(std::memory_order_acquire)) {
        throw ConfigError("Master::sdo_read: refused during a declared RT phase (slave " + std::to_string(slave) + " object " +
                          std::to_string(index) + ":" + std::to_string(sub) +
                          ") -- SDO is pre-RT-spawn/post-RT-join only; steady-state access is the #22 queue");
    }
    return backend_->sdo_read(slave, index, sub, out);
}

// --- #22 steady-state SDO: marshaled through the RT thread -------------------------

void Master::set_rt_active(bool active) noexcept {
    rt_active_.store(active, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lk(sdo_mtx_);
        sdo_service_open_ = active;
        if (!active) {
            // The RT servicer is gone (called after the join). Any in-flight request can no
            // longer be serviced: drop it to Idle and clear pending, so a blocked waiter wakes
            // (its predicate also checks !sdo_service_open_) and throws cleanly rather than
            // hanging across stop()/reconfigure().
            sdo_phase_ = SdoPhase::Idle;
            sdo_pending_.store(false, std::memory_order_release);
        }
    }
    sdo_cv_.notify_all();
}

std::size_t Master::sdo_read_deferred(
    std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out, std::chrono::milliseconds timeout) {
    const std::string what = "slave " + std::to_string(slave) + " object " + std::to_string(index) + ":" + std::to_string(sub);
    std::unique_lock<std::mutex> lk(sdo_mtx_);
    if (!sdo_service_open_) {
        throw ConfigError("Master::sdo_read_deferred: no RT servicer running (" + what +
                          ") -- a steady-state SDO read requires the RT loop; call while operational");
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    // Serialize concurrent submitters: wait for the single slot to be free.
    if (!sdo_cv_.wait_until(lk, deadline, [&] { return sdo_phase_ == SdoPhase::Idle || !sdo_service_open_; })) {
        throw SdoError("Master::sdo_read_deferred: timed out waiting for the SDO slot (" + what + ")");
    }
    if (!sdo_service_open_) {
        throw ConfigError("Master::sdo_read_deferred: RT servicer stopped before the request was posted (" + what + ")");
    }
    // Post the request.
    sdo_job_ = SdoJob{};
    sdo_job_.slave = slave;
    sdo_job_.index = index;
    sdo_job_.sub = sub;
    sdo_job_.want = out.size();
    sdo_phase_ = SdoPhase::Requested;
    sdo_pending_.store(true, std::memory_order_release);  // publish the job fields to the RT reader (acquire in service_sdo)
    // Wait for the RT thread to complete it. service_sdo() holds sdo_mtx_ ACROSS its blocking
    // transfer, so once we re-acquire here the phase is a settled Requested/Done -- never a
    // mid-transfer state, so a timeout can safely reclaim a still-Requested slot.
    (void)sdo_cv_.wait_until(lk, deadline, [&] { return sdo_phase_ == SdoPhase::Done || !sdo_service_open_; });
    if (sdo_phase_ != SdoPhase::Done) {
        const bool closed = !sdo_service_open_;
        if (sdo_phase_ == SdoPhase::Requested) {
            // The RT thread never claimed it (else phase would be Done -- it transitions
            // Requested->Done atomically under this same lock). Reclaim the slot.
            sdo_phase_ = SdoPhase::Idle;
            sdo_pending_.store(false, std::memory_order_release);
            sdo_cv_.notify_all();
        }
        lk.unlock();
        if (closed) {
            throw ConfigError("Master::sdo_read_deferred: RT servicer stopped before the SDO completed (" + what + ")");
        }
        throw SdoError("Master::sdo_read_deferred: SDO read timed out after " + std::to_string(timeout.count()) + " ms (" + what + ")");
    }
    // Done: consume the result and free the slot for the next submitter.
    const bool ok = sdo_job_.ok;
    const std::string err = sdo_job_.error;
    const std::size_t got = sdo_job_.got;
    std::array<std::byte, kMaxSdoReadBytes> buf = sdo_job_.buf;
    sdo_phase_ = SdoPhase::Idle;
    sdo_pending_.store(false, std::memory_order_release);
    lk.unlock();
    sdo_cv_.notify_all();  // wake any submitter waiting on the slot
    if (!ok) {
        throw SdoError(err);
    }
    const std::size_t n = std::min(got, out.size());
    std::memcpy(out.data(), buf.data(), n);
    return n;
}

void Master::service_sdo() noexcept {
    // RT idle fast path: a single acquire-load; pairs with the submitter's release store of
    // sdo_pending_ so the job fields are visible if we do proceed. No lock/alloc when Idle.
    if (!sdo_pending_.load(std::memory_order_acquire)) {
        return;
    }
    std::unique_lock<std::mutex> lk(sdo_mtx_);
    if (sdo_phase_ != SdoPhase::Requested) {
        // A timed-out/closed waiter reclaimed the slot between our atomic load and the lock.
        sdo_pending_.store(false, std::memory_order_release);
        return;
    }
    // Execute exactly ONE transaction, UNDER the lock: holding it across the blocking mailbox
    // round-trip is what makes the waiter's Requested->Done view atomic (no mid-transfer
    // reclaim). This cycle deliberately overruns by the round-trip (~1-2 ms); the Runner's
    // pacer catch-up + the drive's SM watchdog absorb the single-cycle PD gap (#22 HW-verified).
    SdoJob& j = sdo_job_;
    try {
        const std::size_t want = std::min<std::size_t>(j.want, j.buf.size());
        j.got = backend_->sdo_read(j.slave, j.index, j.sub, std::span<std::byte>(j.buf.data(), want));
        j.ok = true;
    } catch (const std::exception& e) {
        j.got = 0;
        j.ok = false;
        j.error = e.what();  // alloc permitted HERE: this is the bounded jitter window, not the idle hot path
    } catch (...) {
        j.got = 0;
        j.ok = false;
        j.error = "unknown error servicing steady-state SDO read";
    }
    sdo_phase_ = SdoPhase::Done;
    sdo_pending_.store(false, std::memory_order_release);
    lk.unlock();
    sdo_cv_.notify_all();
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
