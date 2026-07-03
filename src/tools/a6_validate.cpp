// a6_validate -- TEMPORARY hardware bring-up validation for the A6-EC servo.
//
// Exercises the ethercat master library against a REAL A6 drive in safe stages:
//   Stage A (always):  open NIC -> enumerate -> SDO identity + key objects
//                       (PRE-OP, read-only; no PDO config, no motion).
//   Stage B (always):  configure (PDO remap + 0x6060 mode + OP) -> cyclic
//                       process() -> drive controlword to READY-TO-SWITCH-ON
//                       (motor NOT energized) and read live feedback.
//   --enable           additionally step the CiA402 ladder to OPERATION-ENABLED
//                       and HOLD at the current position (motor energizes,
//                       holding torque, NO commanded motion).
//   --move-pp R [RPM]  after enabling, command a RELATIVE PP move of R revs at
//                       RPM (default 60) via the bit4 new-setpoint handshake and
//                       watch convergence. *** MOTION -- opt-in only. ***
//   --move-sine        after enabling, stream a CSP soft-started sine.
//                       *** MOTION -- opt-in only. ***
//
// #47 P2: this tool now runs on ethercat::Runner. The hand-rolled Phase-1 pump
// (run_to_operational + observer), the Phase-2 steady loop, the local DcPacer,
// and the two teardown loops are DELETED -- the Runner owns the RT thread,
// realtime setup, the one pacer, bring-up, the steady cadence, the stopping
// window, and master.close(). What remains HERE is pure POLICY:
//   - A6Control::step()        = the old Phase-2 CiA402 branch tree, verbatim
//   - A6Control::sync_faulted() = the old 0x603F==0x8700 bring-up gate (A6
//                                 knowledge stays in the CONSUMER -- #41 layering)
//   - the stopping window      = the old graceful teardown (CSP hold-then-disable)
//   - main()                   = a NON-RT printer polling Runner status + the
//                                 control's atomic telemetry (the old in-loop
//                                 prints, moved off the RT thread)
//
// Defaults are non-energizing and motionless. Runtime needs CAP_NET_RAW (raw
// socket). NOT a production path; the real driver is the Viam module.

#include <array>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "ethercat/cia402.hpp"
#include "ethercat/errors.hpp"
#include "ethercat/master.hpp"
#include "ethercat/pdo_buffer.hpp"
#include "ethercat/pdo_mapping.hpp"
#include "ethercat/runner.hpp"
#include "ethercat/soem_backend.hpp"
#include "tools/a6_control.hpp"  // #53: A6Control + Options + Telemetry + the A6 object constants
#include "tools/sine_move.hpp"

namespace {

using namespace ethercat;
using namespace ethercat::tools;  // #53: Options/Telemetry/A6Control + the k* object constants live here now

// The A6 object-index constants, kCountsPerRev/kLoopHz, sdo_value<>(), Options,
// Telemetry, and A6Control moved to tools/a6_control.hpp (#53) so the energized PP/PV
// control policy is offline-testable. main() + build_a6_config() + the SIGINT relay stay.

std::atomic<bool> g_stop{false};
extern "C" void on_sigint(int) {
    g_stop.store(true);
}

// Build the MasterConfig for one A6, mirroring etc/a6-hardware.example.json (RxPDO
// 0x1600 = ctrl + target-pos + profile-vel; TxPDO 0x1A00 = fault + status +
// mode-display + pos + vel + torque). `mode` selects 0x6060: ProfilePosition (the
// bit4-handshake --move-pp) or CyclicSyncPosition (the streamed --move-sine). Both
// reuse the SAME PDO map -- 0x607A serves the PP target AND the CSP streamed target --
// so one builder covers both; only the post-enable control semantics differ.
MasterConfig build_a6_config(const std::string& ifname, Cia402Mode mode, bool use_dc = true) {
    MasterConfig cfg;
    cfg.ifname = ifname;
    cfg.target_loop_rate_hz = kLoopHz;  // 1 ms SYNC0 = 4 x 250 us (A6-legal)
    // #71 bench check: --no-dc requests OP under FREE-RUN. The A6 supports ONLY DC sync, so it
    // refuses with AL 0x0027 "Freerun not supported" -- exercises the AL-status give-up diagnostic.
    cfg.use_distributed_clocks = use_dc;  // A6 supports ONLY DC sync (true); --no-dc forces the free-run refusal
    cfg.dc_settle_cycles = 1000;          // ~1 s post-OP grace while the phase finishes locking
    cfg.max_consecutive_wkc_errors = 5;
    // The bring-up SETTLE bound uses MasterConfig's default (dc_op_gate_cycles). SYNC0 is
    // armed in PRE-OP inside configure() (before config_map_group); the Runner's bring-up
    // pump then runs SETTLE (phase-locked PD) -> request OP once -> AWAIT_OP, all gapless.

    SlaveConfig a6;
    a6.slave_id = 1;
    a6.default_mode = mode;  // 0x6060 set in configure(); PP=1 (handshake) or CSP=8 (streamed sine)
    // #44: the A6 accepts only 250 us-multiple SYNC0 cycles (else Er74.0 at OP entry).
    a6.sync_cycle_granularity_ns = 250'000;
    // #39: the vendor fault-reset is CONSUMER policy -- this tool runs it itself
    // post-configure via Master::sdo_write (see --reset-fault in main), BEFORE
    // Runner::start() (still the single port owner; the #39 bracket is Runner-owned).
    // #20: we DELIBERATELY do NOT write 0x1C32:01 (SM sync-type) -- CLAUDE.md.

    a6.rxpdo.pdo_indices = {0x1600};
    a6.rxpdo.entries[0x1600] = {
        {kControlword, 0, 16},
        {kTargetPosition, 0, 32},
        {kProfileVelocity, 0, 32},
        {kTargetVelocity, 0, 32},  // #53: superset RxPDO -- mapped for the PV mode (target consumed by the held PV path); appended so
                                   // PP/CSP offsets are unchanged
        {kModeOfOperation, 0, 8},  // #47-P3b 5a (P3c): mode-of-operation in the RxPDO so the runtime PP<->PV mode-switch can write 0x6060
                                   // cyclically (14->15 B; P3c HW step-1 = bring-up re-verify with this map)
    };

    a6.txpdo.pdo_indices = {0x1A00};
    a6.txpdo.entries[0x1A00] = {
        {kFaultCode, 0, 16},
        {kStatusword, 0, 16},
        {kModeDisplay, 0, 8},
        {kPositionActual, 0, 32},
        {kVelocityActual, 0, 32},
        {kTorqueActual, 0, 16},
    };

    cfg.slaves.push_back(std::move(a6));
    return cfg;
}

// #72: run ONE full bring-up -> hold -> teardown lifecycle. --cycle repeats this back-to-back on the
// SAME NIC, each iteration constructing a FRESH Master + Runner and destroying them (the in-place
// reconfigure the module does when its config changes). A re-bring-up that leaves DC marginal shows up
// as a mid-hold badWKC climb / OP loss (StopReason::BusFault) on cycle >= 2, or a re-bring-up that no
// longer reaches OP. `hold`.count()==0 => run until SIGINT/abort (the single-run default, unchanged).
struct CycleOutcome {
    StopReason reason = StopReason::None;
    std::uint64_t bad_cycles = 0;
    std::uint64_t total_cycles = 0;
    std::uint16_t al_code = 0;
    bool reached_op = false;
};

CycleOutcome run_one_cycle(const Options& opt, Cia402Mode mode, bool no_dc,
                           std::chrono::seconds hold, int cyc, int ncycles) {
    CycleOutcome oc;
    const std::uint16_t slave = 1;

    Master master(build_a6_config(opt.ifname, mode, /*use_dc=*/!no_dc), std::make_unique<SoemBackend>());

    // --- Stage A: open + enumerate (re-init on cycle >= 2 re-opens the NIC after the prior close()).
    try {
        master.init();
    } catch (const Error& e) {
        std::cerr << "init failed: " << e.what() << '\n';
        oc.reason = StopReason::RtSetupFailed;
        return oc;
    }
    std::cout << "[A] bus up: A6 enumerated (1 slave, matches config).\n";
    const SlaveInfo info = master.slave_info(1);
    std::cout << "    identity: vendor=0x" << std::hex << info.vendor_id << " product=0x" << info.product_code << " rev=0x" << info.revision
              << std::dec << " name=\"" << info.name << "\" (Rx " << info.output_bytes << "B / Tx " << info.input_bytes << "B)\n"
              << "[A] OK -- EtherCAT enumeration confirmed.\n\n";

    // --- Stage B: configure to SAFE-OP + DC; the Runner owns everything after start().
    try {
        master.configure();
    } catch (const Error& e) {
        std::cerr << "[B] configure failed: " << e.what() << "\n"
                  << "    (an AL-reject at the SAFE-OP transition lands here; the AL status code in the message names why.)\n";
        oc.reason = StopReason::BringupAborted;
        return oc;
    }
    std::cout << "[B] configured to SAFE-OP, DC enabled (expected WKC=" << master.expected_wkc()
              << "); handing the bus to the Runner (bring-up pump: SETTLE -> request OP -> AWAIT_OP)...\n";

    if (opt.reset_fault) {
        try {
            std::array<std::byte, 2> fc_raw{};
            const std::size_t n = master.sdo_read(slave, kFaultCode, 0, fc_raw);
            const std::uint16_t fc0 = n >= 2 ? load_le<std::uint16_t>(fc_raw) : 0;
            if (fc0 != 0) {
                std::cout << "[B] latent drive fault 0x" << std::hex << fc0 << std::dec
                          << " -- clearing via vendor SDO 0x2031:01=1 (consumer-side, #39)...\n";
                master.sdo_write(slave, 0x2031, 0x01, sdo_value<std::uint16_t>(1));
            } else {
                std::cout << "[B] --reset-fault: no latent fault (0x603F=0), nothing to clear.\n";
            }
        } catch (const Error& e) {
            std::cerr << "[B] --reset-fault SDO failed (continuing; the bring-up gate still guards OP): " << e.what() << '\n';
        }
    }

    Telemetry tel;
    A6Control control(opt, tel, mode);
    RunnerConfig rc;
    rc.rt_priority = 80;
    rc.require_realtime = false;
    rc.bringup_timeout = std::chrono::milliseconds(120'000);
    rc.teardown_cycles = 150;
    if (opt.move_vel) {
        rc.teardown_cycles = kPvTeardownCycles;
    }

    StopReason reason = StopReason::None;
    std::string al_msg;
    {
        Runner runner(master, rc);
        try {
            runner.attach(slave, control);
            runner.start();
        } catch (const Error& e) {
            std::cerr << "[B] runner start failed: " << e.what() << '\n';
            master.close();
            oc.reason = StopReason::RtSetupFailed;
            return oc;
        }

        const auto decode_modes = [](std::uint32_t bits) {
            static const std::pair<unsigned, const char*> kBits[] = {
                {0, "PP"}, {1, "VL"}, {2, "PV"}, {3, "TQ"}, {5, "HM"}, {6, "IP"}, {7, "CSP"}, {8, "CSV"}, {9, "CST"}};
            std::string out;
            for (const auto& [bit, name] : kBits) {
                if ((bits & (1U << bit)) != 0U) {
                    out += (out.empty() ? "" : ",");
                    out += name;
                }
            }
            return out.empty() ? std::string("(none)") : out;
        };

        auto last_print = std::chrono::steady_clock::now() - std::chrono::seconds(1);
        auto last_sdo = std::chrono::steady_clock::now();
        // #72 hold tracking: mark when Running first begins, then hold a fixed wall time and emit a
        // per-minute health line (badWKC delta since OP + dcPhase) so a slow DC drift is visible.
        bool saw_running = false;
        std::chrono::steady_clock::time_point op_start;
        std::uint64_t bad_at_op = 0;
        auto last_health = std::chrono::steady_clock::now();
        while (runner.status().phase != RunnerPhase::Stopped) {
            if (g_stop.load()) {
                runner.request_stop();  // SIGINT -> graceful stop (the window runs the disable policy)
            }
            const RunnerStatus st0 = runner.status();
            if (st0.phase == RunnerPhase::Running && !saw_running) {
                saw_running = true;
                oc.reached_op = true;
                op_start = std::chrono::steady_clock::now();
                bad_at_op = tel.bad_wkc.load(std::memory_order_relaxed);
                last_health = op_start;
                if (hold.count() > 0) {
                    std::cout << "[cycle " << cyc << "/" << ncycles << "] OP reached; holding " << hold.count()
                              << "s, per-minute health below...\n";
                }
            }
            if (hold.count() > 0 && saw_running && st0.phase == RunnerPhase::Running) {
                const auto now2 = std::chrono::steady_clock::now();
                const auto held = std::chrono::duration_cast<std::chrono::seconds>(now2 - op_start);
                if (now2 - last_health >= std::chrono::seconds(60)) {
                    last_health = now2;
                    const Status status{tel.sw.load(std::memory_order_relaxed)};
                    const std::uint64_t bad_now = tel.bad_wkc.load(std::memory_order_relaxed);
                    std::cout << "[cycle " << cyc << "/" << ncycles << "] +" << held.count() << "s OP-HOLD: "
                              << to_string(status.decode()) << " sw=0x" << std::hex << status.raw << std::dec
                              << " 0x603F=0x" << std::hex << tel.fc.load(std::memory_order_relaxed) << std::dec
                              << " badWKC=" << bad_now << " (+" << (bad_now - bad_at_op) << " since OP)"
                              << " dcPhase=" << tel.dc_phase_ns.load(std::memory_order_relaxed) << "ns\n";
                }
                if (held >= hold) {
                    runner.request_stop();  // #72: hold elapsed -> teardown -> next cycle re-brings-up
                }
            }
            // #22 mid-run direct non-RT SDO reads (only while Running; #15 -- the caller drives the mailbox exchange).
            if (opt.sdo_probe && runner.status().phase == RunnerPhase::Running &&
                std::chrono::steady_clock::now() - last_sdo >= std::chrono::milliseconds(500)) {
                last_sdo = std::chrono::steady_clock::now();
                std::string line = "[sdo] ";
                {
                    std::array<std::byte, 4> vbuf{};
                    try {
                        const std::size_t n = master.sdo_read(slave, kDcLinkVoltage, 0, vbuf);
                        line += "0x6079 DC-link=" + std::to_string((n >= 4 ? load_le<std::uint32_t>(vbuf) : 0) / 1000.0) + "V";
                    } catch (const Error& e) {
                        line += std::string("0x6079 ERR{") + e.what() + "}";
                    }
                }
                {
                    std::array<std::byte, 2> cbuf{};
                    try {
                        const std::size_t n = master.sdo_read(slave, kCurrentActual, 0, cbuf);
                        line += " | 0x6078 current=" + std::to_string(n >= 2 ? load_le<std::int16_t>(cbuf) : 0) + "permille";
                    } catch (const Error& e) {
                        line += std::string(" | 0x6078 ERR{") + e.what() + "}";
                    }
                }
                {
                    std::array<std::byte, 4> mbuf{};
                    try {
                        const std::size_t n = master.sdo_read(slave, kSupportedModes, 0, mbuf);
                        const std::uint32_t modes = n >= 4 ? load_le<std::uint32_t>(mbuf) : 0;
                        char hex[16];
                        (void)std::snprintf(hex, sizeof(hex), "0x%X", modes);
                        line += " | 0x6502 modes=" + std::string(hex) + " {" + decode_modes(modes) + "}";
                    } catch (const Error& e) {
                        line += std::string(" | 0x6502 ERR{") + e.what() + "}";
                    }
                }
                line += " badWKC=" + std::to_string(tel.bad_wkc.load(std::memory_order_relaxed));
                std::cout << line << '\n';
            }
            const auto now = std::chrono::steady_clock::now();
            if (now - last_print >= std::chrono::milliseconds(200)) {  // ~5 Hz
                last_print = now;
                const RunnerStatus st = runner.status();
                if (st.phase == RunnerPhase::BringingUp) {
                    std::cout << "[B] bring-up... dcPhase=" << tel.dc_phase_ns.load(std::memory_order_relaxed) << "ns\n";
                } else if (st.phase == RunnerPhase::Running || st.phase == RunnerPhase::Stopping) {
                    const Status status{tel.sw.load(std::memory_order_relaxed)};
                    std::cout << "    t=" << tel.cycle.load(std::memory_order_relaxed) / kLoopHz << "s " << to_string(status.decode())
                              << " sw=0x" << std::hex << status.raw << " 0x603F=0x" << tel.fc.load(std::memory_order_relaxed) << std::dec
                              << " mode=" << static_cast<int>(tel.mode.load(std::memory_order_relaxed)) << " Rx.cw=0x" << std::hex
                              << tel.cw.load(std::memory_order_relaxed) << std::dec
                              << " cmdTarget=" << tel.target.load(std::memory_order_relaxed)
                              << " pos=" << tel.pos.load(std::memory_order_relaxed)
                              << " followErr=" << (tel.target.load(std::memory_order_relaxed) - tel.pos.load(std::memory_order_relaxed))
                              << " vel=" << tel.vel.load(std::memory_order_relaxed)
                              << " badWKC=" << tel.bad_wkc.load(std::memory_order_relaxed)
                              << " dcPhase=" << tel.dc_phase_ns.load(std::memory_order_relaxed) << "ns"
                              << (st.phase == RunnerPhase::Stopping ? " [STOPPING]" : "") << '\n';
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        reason = runner.status().reason;  // read before the dtor teardown
        if (reason == StopReason::BringupAborted) {
            // Prefer the LATCHED last-non-zero AL code from AWAIT (#71/#25): the live al_status_code()
            // can read 0 at the give-up (reack_op ACKs the SAFE_OP+ERROR on the timeout cycle), which
            // is exactly what defeated the first cut of this diagnostic (printed "0x0 No error").
            oc.al_code = master.bringup_al_code();
            if (oc.al_code == 0) {  // no non-zero code was seen the whole bring-up -- fall back to the live read
                oc.al_code = master.al_status_code(slave);
            }
            al_msg = master.describe_al_code(oc.al_code);
        }
    }  // <-- ~Runner: bounded stop -> join -> rt_active(false) -> master.close()

    const WkcStats stats = master.wkc_stats();
    oc.reason = reason;
    oc.bad_cycles = stats.bad_cycles;
    oc.total_cycles = stats.total_cycles;
    std::cout << "\n=== cycle " << cyc << "/" << ncycles << " done. stop=" << to_string(reason)
              << (control.safety_abort() ? " (CSP SAFETY ABORT)" : "")
              << " | bad-WKC cycles: " << stats.bad_cycles << " / " << stats.total_cycles << " ===\n";
    if (reason == StopReason::BringupAborted) {
        std::cout << "[#71] bring-up gave up. AL status = 0x" << std::hex << oc.al_code << std::dec << " (" << al_msg << ")"
                  << (oc.al_code == 0x0027 ? " -- FREERUN NOT SUPPORTED: this drive requires DC (drop --no-dc)" : "") << '\n';
    }
    return oc;
}  // <-- master dtor here (close already ran in ~Runner); NIC port released for the next cycle's init()

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    bool no_dc = false;  // #71: --no-dc -> free-run bring-up (the A6 refuses OP; AL-status give-up check)
    std::vector<std::string> args(argv + 1, argv + argc);
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        if (a == "--enable") {
            opt.enable = true;
        } else if (a == "--no-dc") {
            no_dc = true;  // #71: bring up under free-run (no SYNC0) -> the A6 refuses OP (AL 0x0027)
        } else if (a == "--reset-fault") {
            opt.reset_fault = true;
        } else if (a == "--move-pp" && i + 1 < args.size()) {
            opt.move_pp = true;  // requires an EXPLICIT --enable (checked below) -- no implicit energize
            opt.move_revs = std::stod(args[++i]);
            if (i + 1 < args.size() && args[i + 1].rfind("--", 0) != 0) {
                opt.move_rpm = std::stod(args[++i]);
            }
        } else if (a == "--move-pos" && i + 1 < args.size()) {
            opt.move_pos = true;  // #53 absolute PP move-to; requires --enable (checked below)
            opt.pos_target = std::stoi(args[++i]);
            if (i + 1 < args.size() && args[i + 1].rfind("--", 0) != 0) {
                opt.pp_vel_cps = std::stoi(args[++i]);  // optional profile velocity (counts/s)
            }
        } else if (a == "--move-vel" && i + 1 < args.size()) {
            opt.move_vel = true;  // #53 continuous PV until Ctrl-C; requires --enable
            opt.pv_vel_cps = std::stoi(args[++i]);
        } else if (a == "--then-jog-vel" && i + 1 < args.size()) {
            opt.then_jog_vel = true;  // #47-P3b P3c: after the --move-pos reaches, SWITCH PP->PV (§6) + jog at VEL until Ctrl-C
            opt.pv_vel_cps = std::stoi(args[++i]);
        } else if (a == "--pos-tol" && i + 1 < args.size()) {
            opt.pos_tol = std::stoi(args[++i]);  // #53 DA-C: reached tolerance (counts); default 300
        } else if (a == "--move-sine") {
            opt.move_sine = true;  // requires an EXPLICIT --enable (checked below) -- no implicit energize
        } else if (a == "--sdo-probe") {
            opt.sdo_probe = true;  // #22: read 0x6079/0x6078/0x6502 via the direct non-RT steady-state SDO while Running (#15)
        } else if (a == "--csp-probe") {
            opt.csp_probe = true;  // CSP mode, NO enable -- read+print feedback only (diagnostic)
        } else if (a == "--sine-amplitude" && i + 1 < args.size()) {
            opt.sine_amplitude = std::stod(args[++i]);
        } else if (a == "--sine-period" && i + 1 < args.size()) {
            opt.sine_period = std::stod(args[++i]);
        } else if (a == "--follow-err-limit" && i + 1 < args.size()) {
            opt.follow_err_limit = std::stoi(args[++i]);
        } else if (a == "--cycle" && i + 1 < args.size()) {
            opt.cycle_count = std::stoi(args[++i]);  // #72: N back-to-back reconfigure cycles
        } else if (a == "--hold-seconds" && i + 1 < args.size()) {
            opt.hold_seconds = std::stoi(args[++i]);  // #72: final-cycle soak seconds
        } else if (a == "--early-hold" && i + 1 < args.size()) {
            opt.early_hold_seconds = std::stoi(args[++i]);  // #72: per-early-cycle hold seconds
        } else if (a.rfind("--", 0) != 0) {
            opt.ifname = a;
        } else {
            std::cerr << "usage: a6_validate [ifname] [--enable] [--reset-fault] [--move-pp REVS [RPM]]\n"
                      << "                   [--move-pos POS [VEL]] [--move-vel VEL] [--pos-tol N]\n"
                      << "                   [--move-sine [--sine-amplitude N] [--sine-period S]] [--csp-probe]\n"
                      << "  --move-pos POS [VEL]: *** MOTION (needs --enable) *** absolute PP move-to POS counts at VEL\n"
                      << "               counts/s (profile vel; default from --move-pp RPM if omitted). Reached = |POS-actual|\n"
                      << "               <= --pos-tol (default 300 counts, #53 DA-C) AND velocity ~0; then holds.\n"
                      << "  --move-pos POS --then-jog-vel VEL: *** MOTION (needs --enable) *** move to POS (PP), then at\n"
                      << "                   reach SWITCH PP->PV (runtime 0x6060 mode-switch, P3c) and jog at VEL counts/s until Ctrl-C\n"
                      << "  --move-vel VEL: *** CONTINUOUS MOTION (needs --enable) *** Profile-Velocity at VEL counts/s until\n"
                      << "               Ctrl-C. On stop: CiA402 Quick-Stop (cw=0x0B) -> drive ramps via 0x6085 -> de-energizes\n"
                      << "               at zero (requires 0x605A=2, asserted at configure; 0x6085 written + readback-checked).\n"
                      << "  The DC bring-up is automatic (Runner-owned, #47): configure() arms SYNC0 in PRE-OP, the\n"
                      << "  Runner's pump runs SETTLE -> request OP once -> AWAIT_OP, gapless + phase-locked.\n"
                      << "  Er74.1 in SAFE-OP is normal pre-sync, clears at OP.\n"
                      << "  --enable: energize to OperationEnabled (holding torque). REQUIRED for any move below --\n"
                      << "            --move-pp/--move-sine no longer imply it, so a forgotten --enable fails closed.\n"
                      << "  --move-sine: *** ENERGIZED MOTION (needs --enable) *** CSP-mode soft-started position sine,\n"
                      << "               relative to the enable position. pos(t)=pos_enable + A*min(1,t/T)*sin(2*pi*t/T);\n"
                      << "               A=--sine-amplitude (counts, def 20000), T=--sine-period (s, def 4.0). CSP-safe\n"
                      << "               (no jump) + ramped (no velocity step). --follow-err-limit N (counts, def 5000):\n"
                      << "               abort+disable if |commanded-actual| exceeds it. Mutually exclusive with --move-pp.\n"
                      << "  --move-pp REVS [RPM]: *** MOTION (needs --enable) *** PP-mode relative move via the bit4 handshake.\n"
                      << "  --csp-probe: NON-energizing diagnostic -- bring up in CSP mode (0x6060=8), hold at\n"
                      << "               ReadyToSwitchOn (NO enable), print feedback.\n"
                      << "  --sdo-probe: #22 steady-state SDO -- while Running, read 0x6079 (DC-link V), 0x6078 (current),\n"
                      << "               0x6502 (supported modes) every ~500ms via the direct non-RT SDO path (#15); print\n"
                      << "               raw + converted. Safe with a plain hold (no --enable); exercises mid-run mailbox reads.\n"
                      << "  --reset-fault: clear a latent drive fault at bring-up via the A6 vendor SDO 0x2031:01=1.\n"
                      << "  --cycle N [--early-hold S] [--hold-seconds S]: #72 in-place-reconfigure repro -- run N\n"
                      << "               back-to-back bring-up->hold->teardown lifecycles on the same NIC (no power cycle,\n"
                      << "               the module's reconfigure). Early cycles hold --early-hold s (def 20); the LAST holds\n"
                      << "               --hold-seconds s (def 600 = 10min soak). Per-minute WKC/dcPhase health line; a\n"
                      << "               mid-hold WKC drop-out or a re-bring-up that won't reach OP is flagged [#72].\n";
            return 2;
        }
    }

    const int mode_flags = mode_flag_count(opt);
    if (mode_flags > 1) {
        std::cerr << "error: --move-pp / --move-pos / --move-vel / --move-sine / --csp-probe are mutually exclusive "
                     "(one mode of operation at a time)\n";
        return 2;
    }
    // SAFETY: a move must NOT silently energize. The move flags require an explicit
    // --enable, so a forgotten --enable FAILS CLOSED instead of moving the shaft.
    if ((opt.move_pp || opt.move_sine || opt.move_pos || opt.move_vel) && !opt.enable) {
        std::cerr << "error: --move-pp / --move-pos / --move-vel / --move-sine command ENERGIZED MOTION and require an "
                     "explicit --enable\n"
                  << "       (safety: motion must be a deliberate opt-in -- a forgotten --enable will not silently move the shaft).\n"
                  << "       For a non-energizing CSP feedback read, use --csp-probe instead.\n";
        return 2;
    }
    const Cia402Mode mode = opt.move_vel                       ? Cia402Mode::ProfileVelocity
                            : (opt.move_sine || opt.csp_probe) ? Cia402Mode::CyclicSyncPosition
                                                               : Cia402Mode::ProfilePosition;  // move_pos / move_pp / plain hold

    (void)std::signal(SIGINT, on_sigint);

    std::cout << "=== A6-EC validation on '" << opt.ifname << "' (Runner-based, #47 P2) ===\n"
              << "mode: " << to_string(mode) << " | enable=" << (opt.enable ? "YES (motor energizes)" : "no") << " | move="
              << (opt.move_sine ? "SINE A=" + std::to_string(static_cast<long>(opt.sine_amplitude)) +
                                      "ct T=" + std::to_string(opt.sine_period) + "s (CSP, soft-started)"
                  : opt.move_vel ? "VEL " + std::to_string(opt.pv_vel_cps) + " counts/s (PV, until Ctrl-C)"
                  : opt.move_pos ? "POS " + std::to_string(opt.pos_target) + " counts @ " +
                                       std::to_string(opt.pp_vel_cps > 0 ? opt.pp_vel_cps : 0) + " counts/s (PP move-to)"
                  : opt.move_pp ? std::to_string(opt.move_revs) + " rev @ " + std::to_string(opt.move_rpm) + " rpm (PP)"
                                : "none (hold)")
              << "\n\n";

    // DC SYNC0 cycle = loop period; the A6 requires an integer multiple of 250 us.
    constexpr std::uint32_t kCycleNs = 1'000'000'000U / kLoopHz;
    static_assert(kCycleNs % 250'000U == 0, "SYNC0 cycle must be a 250us multiple for the A6");

    std::cout << "[rt] realtime setup (mlockall/SCHED_FIFO prio 80) is Runner-owned now (#47); best-effort\n"
              << "     (require_realtime=false) -- run with sudo for DC-safe timing.\n"
              << "[dc] phase-lock target = auto(mid-cycle) | SYNC0 CyclShift = 0ns (config knob)"
              << " | fault-reset at bring-up = " << (opt.reset_fault ? "ON (vendor 0x2031:01=1)" : "off") << "\n\n";

    const int ncycles = std::max(1, opt.cycle_count);
    const std::chrono::seconds early_hold{opt.early_hold_seconds};
    const std::chrono::seconds final_hold{opt.hold_seconds};
    CycleOutcome last;
    int worst_rc = 0;
    for (int cyc = 1; cyc <= ncycles; ++cyc) {
        if (opt.cycle_count > 0) {
            std::cout << "\n========== CYCLE " << cyc << "/" << ncycles
                      << (cyc == 1 ? " (fresh bring-up)"
                                   : " (in-place re-bring-up, NO power cycle -- #72 reconfigure repro)")
                      << " ==========\n";
        }
        // #72: --cycle holds each cycle a fixed wall time -- short on early cycles, the long soak on the
        // last -- then tears down and re-brings-up. Single-run (no --cycle) => hold 0 = run until
        // SIGINT/abort (unchanged behavior).
        const std::chrono::seconds hold = opt.cycle_count == 0 ? std::chrono::seconds{0}
                                          : (cyc == ncycles ? final_hold : early_hold);
        last = run_one_cycle(opt, mode, no_dc, hold, cyc, ncycles);
        if (last.reason == StopReason::BringupAborted || last.reason == StopReason::RtSetupFailed) {
            worst_rc = 1;
        }
        if (last.reason == StopReason::BusFault) {
            worst_rc = 1;
            std::cout << "[#72] *** DROP-OUT on cycle " << cyc << "/" << ncycles
                      << ": bus fault (WKC latch) mid-hold -- the reconfigure-DC-drift signature. ***\n";
        }
        if (opt.cycle_count > 0 && cyc >= 2 && !last.reached_op) {
            worst_rc = 1;
            std::cout << "[#72] *** re-bring-up on cycle " << cyc << "/" << ncycles
                      << " never reached OP (stop=" << to_string(last.reason)
                      << ") -- residual DC/config poisoning from the prior teardown. ***\n";
        }
        if (g_stop.load()) {
            std::cout << "[cycle] SIGINT -- stopping the cycle loop.\n";
            break;
        }
        if (cyc < ncycles) {
            // Brief settle between teardown and the next init() -- the A6's enumeration is intermittent
            // right after a close() (a known retry quirk); this does NOT mask the DC drift, which shows
            // during the multi-minute OP hold, not at enumerate.
            std::cout << "[cycle] teardown complete; re-bring-up next (no power cycle)...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    if (opt.cycle_count > 0) {
        std::cout << "\n========== --cycle SUMMARY: " << ncycles << " cycles, final stop="
                  << to_string(last.reason) << " ==========\n";
    }
    return worst_rc;
}