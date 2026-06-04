#include "ethercat/master.hpp"

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string>
#include <utility>

#include <sys/mman.h>

#include "ethercat/cia402.hpp"

namespace ethercat {

namespace {

constexpr std::uint16_t kModesOfOp = 0x6060;  // CiA402 modes-of-operation (U8): PP=1, PV=3; SDO-set in PRE-OP
constexpr long kNsPerSec = 1'000'000'000L;
// AWAIT_OP bounds (bringup_step): require this many consecutive (full-WKC && no-Er74.1)
// cycles to confirm OP is reached + synced (not a transient), and give up after this many
// cycles without that hold -- a SINGLE OP request (ec_sample does it with Er74.1 present),
// bounded, never re-requested (hammering OP-entry wedges the A6, CLAUDE.md).
constexpr std::uint32_t kOpHoldConfirm = 5;   // ~5 ms @ 1 kHz of held sync -> Operational
constexpr std::uint32_t kAwaitOpBound = 500;  // ~500 ms to reach the held-synced state, else Aborted

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

void Master::configure() {
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
        // Lock resident memory (IOmap + SOEM context) before the RT thread spawns and
        // starts pacing SYNC0, so a page fault never spikes the phase. MCL_CURRENT only
        // (NOT MCL_FUTURE: the RT jthread's stack alloc would otherwise hit
        // RLIMIT_MEMLOCK -> EAGAIN).
        if (mlockall(MCL_CURRENT) != 0) {
            (void)std::fprintf(stderr,
                               "[ethercat] mlockall(MCL_CURRENT) failed (errno=%d): grant CAP_IPC_LOCK / "
                               "RLIMIT_MEMLOCK=infinity; the SYNC0 PLL lock may be unreliable.\n",
                               errno);
        }
    }

    // Stop at SAFE-OP. SYNC0 is armed (PRE-OP) but its first edge is ~100 ms out (stock
    // SyncDelay), so the brief PRE-OP->SAFE-OP statecheck (no PD) finishes well before it
    // -- the RT loop is pumping phase-locked PD before the first pulse. The drive latched
    // DC sync-type at this transition (SYNC0 already armed).
    backend_->request_state(0, EcatState::SafeOp);

    // Clear a latent drive fault while still the SINGLE port owner (before the RT thread
    // spawns) -- a direct blocking vendor SDO. The A6's fault-reset is 0x2031:01 = 1, NOT
    // CiA402 controlword bit7 (CLAUDE.md); it is CONFIG DATA (absent ⇒ no vendor reset,
    // a generic CiA402 drive uses the bit7 path the controller drives). Best-effort: a
    // failed clear is logged, not fatal (the bring-up gate still guards OP entry).
    for (const SlaveConfig& sc : config_.slaves) {
        if (sc.fault_reset.has_value()) {
            const SdoWrite& fr = *sc.fault_reset;
            try {
                backend_->sdo_write(sc.slave_id, fr.index, fr.subindex, fr.data);
            } catch (const Error& e) {
                (void)std::fprintf(stderr,
                                   "[ethercat] bring-up fault-reset SDO (slave %u 0x%04X:%02X) failed (continuing): %s\n",
                                   static_cast<unsigned>(sc.slave_id),
                                   static_cast<unsigned>(fr.index),
                                   static_cast<unsigned>(fr.subindex),
                                   e.what());
            }
        }
    }

    // Hand off to the caller's RT loop at SAFE-OP with SYNC0 already armed. It runs
    // bringup_step() -- SETTLE (bounded phase-locked PD) -> request OP -> AWAIT_OP (hold
    // for OP + Er74.1-cleared + WKC) -- until OPERATIONAL, so operational_ stays false
    // until then. Reset the FSM.
    dc_enabled_ = config_.use_distributed_clocks;
    bringup_phase_ = BringupPhase::Settle;
    bringup_settle_count_ = 0;
    bringup_await_count_ = 0;
    bringup_op_hold_streak_ = 0;
    fault_.store(false, std::memory_order_relaxed);
    consecutive_wkc_errors_ = 0;
    settle_remaining_ = 0;
    operational_.store(false, std::memory_order_relaxed);
}

BringupStatus Master::bringup_step(bool drive_sync_faulted) noexcept {
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
            // backends need no settle (1 cycle). Then request OP ONCE (no hammer).
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
            // OP requested once; pump the gapless transition. Success = full WKC AND no
            // Er74.1 held kOpHoldConfirm cycles -- i.e. the drive reached OP, SYNC0 aligned
            // (Er74.1 cleared), and PD is exchanging cleanly. If that held state isn't
            // reached within kAwaitOpBound cycles, OP didn't take: abort cleanly with NO
            // re-request (a single OP request with Er74.1 present is safe -- ec_sample does
            // it; HAMMERING re-requests is what wedges the A6, CLAUDE.md).
            ++bringup_await_count_;
            if (wkc == expected_wkc_ && !drive_sync_faulted) {
                ++bringup_op_hold_streak_;
            } else {
                bringup_op_hold_streak_ = 0;
            }
            if (bringup_op_hold_streak_ >= kOpHoldConfirm) {
                operational_.store(true, std::memory_order_relaxed);
                settle_remaining_ = dc_enabled_ ? config_.dc_settle_cycles : 0;
                fault_.store(false, std::memory_order_relaxed);
                consecutive_wkc_errors_ = 0;
                bringup_phase_ = BringupPhase::Done;
                return BringupStatus::Operational;
            }
            if (bringup_await_count_ >= kAwaitOpBound) {
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
