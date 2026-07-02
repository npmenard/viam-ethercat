#pragma once

// SimBackend -- an in-memory EtherCAT bus for offline tests (no NIC, no SOEM, no
// SDK). It implements the DEVICE side of CiA402 (the inverse of Cia402Fsm):
// each cyclic exchange consumes the controlword from a slave's RxPDO image,
// advances a toy DS402 device state, and emits the statusword + feedback into
// the TxPDO image. It mirrors the A6-EC where it matters: statusword bit10 is
// held =1 always, and the Profile-Position bit12 set-point-acknowledge
// handshake is modeled.
//
// This is the primary CI integration path: a Master driving a SimBackend
// exercises the full RT loop + CiA402 enable ladder + setpoint propagation +
// snapshot readback with no hardware.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "ethercat/backend.hpp"
#include "ethercat/cia402.hpp"

namespace ethercat {

// Per-slave simulation model. The test supplies it; it stays generic (A6
// specifics are config data, never code). Offsets are byte offsets into the
// respective process-data images.
struct SimSlaveModel {
    std::size_t output_bytes = 0;  // RxPDO command image size
    std::size_t input_bytes = 0;   // TxPDO feedback image size

    std::size_t ctrlword_off = 0;    // 0x6040 in outputs (u16)
    std::size_t statusword_off = 0;  // 0x6041 in inputs (u16)
    std::size_t target_off = 0;      // 0x607A target position in outputs (i32)
    std::size_t actual_off = 0;      // 0x6064 actual position in inputs (i32)

    Cia402Mode mode = Cia402Mode::ProfilePosition;
    std::int32_t counts_per_step = 1000;     // PP: how fast actual chases target per cycle
    std::int32_t velocity_off = -1;          // optional: 0x60FF target velocity offset in outputs (i32); <0 = none
    std::int32_t profile_velocity_off = -1;  // optional: 0x6081 PP profile-velocity offset in outputs (u32); <0 = none
    std::int32_t mode_of_op_off = -1;    // optional: 0x6060 mode-of-operation (i8) offset in OUTPUTS; <0 = mode set by SDO only (#47-P3b
                                         // sub-step 5: RxPDO runtime mode-switch)
    std::int32_t mode_display_off = -1;  // optional: 0x6061 mode-display (i8) offset in INPUTS; <0 = not emitted (#53)

    // --- #53 PV / quick-stop modeling (all default to a conformant A6 that PASSES the gates) ---
    std::int16_t quick_stop_option = 2;       // 0x605A read value (must be 2 for the control to energize PV)
    std::uint32_t quick_stop_decel_echo = 0;  // 0x6085 sdo_read OVERRIDE (clamp / absent model) when *_forced
    bool quick_stop_decel_echo_forced =
        false;  // true -> sdo_read(0x6085) returns quick_stop_decel_echo (0 = absent/refuse, or a clamped value)
    std::int32_t quick_stop_decel_step = 0;  // QuickStopActive: |velocity| ramp-down PER CYCLE (toy counts/cycle); 0 = instant stop
    bool quick_stop_suppress_auto_disable =
        false;  // model a drive that reports 0x605A=2 but does NOT auto-transition QSA->SwitchOnDisabled at zero (forces the control's
                // cw->0x00 BACKSTOP to do the disable -- tests that path while configure still passes)
    std::int8_t mode_echo_value = 0;  // forced 0x6061 echo (wrong-mode refuse test) when mode_echo_forced
    bool mode_echo_forced = false;    // true -> 0x6061 reports mode_echo_value; else it echoes effective_mode (#53 DA-B)
    // --- #47-P3b M56S: a SECOND device's runtime mode-switch quirks (the generic mode-switch's T_switch
    //     knob adapts the SAME policy to these -- no per-device policy code). ---
    std::uint32_t mode_switch_latency = 0;  // cycles the drive TAKES to apply a new 0x6060 (0x6061 lags this many cycles); models M56S
                                            // "transition takes time" (undefined-feedback window). 0 = instant (the A6).
    // #59: encoder READ noise -- the REPORTED 0x6064/0x606C jitter by a ±report_noise square wave (the
    // physics s.actual stays clean). At rest this makes 0x606C nonzero (defeats an exact-|vel|<=0 reached
    // predicate) while the position RANGE stays 2*report_noise (a position-delta predicate tolerates it).
    // 0 = clean (default; all existing tests unaffected).
    std::int32_t report_noise = 0;
    Cia402Mode unsupported_mode = Cia402Mode::None;  // a mode the drive REJECTS (never applies -> 0x6061 never echoes it); models M56S
                                                     // "errors on an unsupported mode". None = accept all.
    // De-mask of the #16 TxPDO FEEDBACK fields (offsets into the INPUT image; <0 = not
    // mapped, so the controller's read path falls back -- exercises the optional guard).
    std::int32_t fault_code_off = -1;       // 0x603F drive error code (u16) in inputs
    std::int32_t velocity_actual_off = -1;  // 0x606C velocity-actual (i32) in inputs

    // Statusword bit10 (target_reached) quirk knob (#43): some drives -- the A6 -- hold
    // bit10 ALWAYS SET, so it can never signal move-complete (the reason the controller's
    // move-complete predicate is |target-actual|<=tol && |vel|<=thresh, NEVER bit10).
    // Default models a CONFORMANT drive (bit10 not forced); A6-shaped test models set it
    // true. Quirk DATA lives here in the model, never hardcoded in the sim (per this
    // header's own principle above).
    bool target_reached_always_set = false;

    // #22 steady-state SDO read targets (served by sdo_read; a real drive holds these in its
    // OD, not the PDO image). Plausible A6-at-hold values so the full do_command -> controller
    // -> marshaled SDO -> backend path runs offline. Standard CiA402 objects:
    //   0x6079:00 U32 DC-link circuit voltage, unit mV  (310 V rectified 220 VAC -> 310000)
    //   0x6078:00 I16 current actual value, per-mille of rated current (0 at hold)
    //   0x6502:00 U32 supported drive modes bitmask (bit0 PP, bit2 PV, bit7 CSP, bit8 CSV)
    std::uint32_t dc_link_voltage_mv = 310'000;          // 0x6079
    std::int16_t current_actual_permille = 0;            // 0x6078
    std::uint32_t supported_drive_modes = 0x0000'0185U;  // 0x6502: PP|PV|CSP|CSV = bits 0,2,7,8

    std::uint32_t vendor_id = 0;
    std::uint32_t product_code = 0;
    std::string name = "sim-slave";
};

class SimBackend final : public EcatBackend {
   public:
    explicit SimBackend(std::vector<SimSlaveModel> slaves);

    // EcatBackend -- setup
    std::size_t open(std::string_view ifname) override;
    SlaveInfo slave_info(std::uint16_t slave) const override;
    void sdo_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<const std::byte> data) override;
    std::size_t sdo_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::span<std::byte> out) override;
    void map_process_data() override;
    void request_state(std::uint16_t slave, EcatState target) override;
    EcatState slave_state(std::uint16_t slave) const override;
    void arm_dc_sync(std::uint32_t cycle_ns, std::int32_t sync0_shift_ns) override;  // records the cycle (configured_dc_cycle_ns)
    std::int64_t dc_time() const noexcept override;  // synthetic ramp (advances per exchange) so phase-lock math is sane offline

    // EcatBackend -- cyclic
    SlaveIo slave_io(std::uint16_t slave) noexcept override;
    int exchange() noexcept override;
    int expected_wkc() const noexcept override;
    void close() noexcept override;

    // --- test hooks (not part of EcatBackend) ---
    // Inject a fault on a slave (next exchange decodes to Fault).
    void inject_fault(std::uint16_t slave) noexcept;
    // Set the drive error code (0x603F) the slave reports WHILE faulted (0 = none).
    // Written to the 0x603F TxPDO offset each cycle the device is in Fault, else 0.
    // Lets a test exercise the #16 drive-fault tier + the code-pending race (set the
    // fault first with code 0, then set the code a cycle later).
    void set_fault_code(std::uint16_t slave, std::uint16_t code) noexcept;
    // Force a STALE 0x603F: write `code` to the 0x603F offset UNCONDITIONALLY (even
    // when the device is NOT in Fault / bit3 clear), modelling a drive that leaves a
    // nonzero error code lingering after the fault clears. 0 = off (normal gated
    // behaviour). Lets the #16 flag-gating test assert that a nonzero stale code with
    // status.fault()==false produces NO drive tier in last_error() -- the literal
    // "flag gates the payload", not a zero-code coincidence.
    void set_stale_fault_code(std::uint16_t slave, std::uint16_t code) noexcept;
    // Type-(a) fault-reset reflect latency (spec #18): on a fault_reset rising edge, the
    // device ACCEPTS the reset but delays the Fault->Switch-On-Disabled reflect by
    // `cycles` exchanges (models real-HW clear latency). 0 = instant (current behaviour).
    // Lets a test reproduce the Enabling-bounce race the instant-clear hides.
    void set_fault_clear_delay(std::uint16_t slave, std::uint32_t cycles) noexcept;
    // Type-(b) persistent-cause (spec #18): while on, a fault_reset rising edge does NOT
    // clear the fault (the cause is still active, e.g. Er74.0) -- the device stays in
    // Fault regardless of bit7. Turn off (then reset) to model the cause being removed.
    void set_fault_persistent(std::uint16_t slave, bool on) noexcept;
    // Type-(c) clear-then-refault (spec #18): on a fault_reset rising edge the device
    // clears MOMENTARILY, then RE-faults after `hold_cycles` exchanges (accepts the reset,
    // resumes, re-detects the cause). hold_cycles < fault_reset_clear_confirm_cycles
    // exercises the debounce -> the clear never CONFIRMS -> give-up. 0 = off.
    void set_fault_clear_then_refault(std::uint16_t slave, std::uint32_t hold_cycles) noexcept;
    // Cumulative count of controlword bit7 (fault-reset) 0->1 rising edges the device has
    // seen (spec #18 no-spin test): snapshot after a give-up, poll N cycles, assert it
    // does NOT climb -> the FSM is not self-re-edging / self-spinning resets.
    std::uint32_t fault_reset_edge_count(std::uint16_t slave) const noexcept;
    // Force the next send_receive() to report a short WKC (one cycle), to test
    // the master's WKC-fault latch.
    void force_short_wkc_once() noexcept;
    // STICKY short WKC: every exchange reports a short WKC while on, so an async RT
    // loop reliably accumulates enough consecutive bad cycles to latch a BUS fault
    // (the #16 compose-both test needs a live master_->fault() without driving cycles).
    void force_short_wkc(bool on) noexcept;
    // Last profile velocity (0x6081) the device saw in its command image (0 if the
    // master never wrote it / it isn't mapped). Lets offline tests assert the RT loop
    // actually writes the commanded move speed.
    // TEST-ONLY: call only AFTER the controller is stopped/joined -- it reads a
    // non-atomic int the RT thread writes via exchange(); a live concurrent read races.
    std::int32_t received_profile_velocity(std::uint16_t slave) const noexcept;
    // #53 PV test visibility (call only AFTER the controller is stopped/joined -- RT-written):
    // last 0x60FF (target velocity) the device saw; the 0x606C velocity at the QuickStopActive ->
    // SwitchOnDisabled transition (proves the ramp reached ~0 BEFORE de-energize); and whether
    // the device ever entered QuickStopActive (proves a CiA402 Quick-Stop, not a torque-cut).
    std::int32_t received_target_velocity(std::uint16_t slave) const noexcept;
    std::uint16_t received_controlword(std::uint16_t slave) const noexcept;  // #47-P3b 5d: last cw consumed (stop-sequence disposition)
    Cia402Mode effective_mode(std::uint16_t slave) const noexcept;  // #47-P3b M6: drive's current runtime mode (PV->PP hold-switch proof)
    std::int32_t received_target_position(std::uint16_t slave)
        const noexcept;  // #47-P3b: last 0x607A the master wrote (DA no-lunge: seeded/mirrored target == actual, never a stale jump)
    std::uint32_t mode_of_op_write_transitions(std::uint16_t slave) const
        noexcept;  // #61 (DA): cumulative 0x6060 OUTPUT-byte changes (unconfirmable-switch storm gate; freezes with revert, climbs without)
    std::int32_t velocity_at_qsa_exit(std::uint16_t slave) const noexcept;
    bool entered_qsa(std::uint16_t slave) const noexcept;
    // Toggle whether a slave asserts the PP set-point-acknowledge (bit12). When
    // suppressed, the controller's new-set-point handshake times out; re-enabling
    // lets a subsequent move complete (handshake-timeout-then-recovery test).
    void suppress_setpoint_ack(std::uint16_t slave, bool on = true) noexcept;
    // SYNC0 cycle (ns) the bring-up armed via arm_dc_sync, or 0 if it never did. Lets an
    // offline test assert the DC bring-up reached the ARM phase when use_distributed_clocks.
    // TEST-ONLY: call only after the controller is stopped/joined (set during the RT loop's
    // bring-up prelude, on the RT thread).
    std::uint32_t configured_dc_cycle_ns() const noexcept;
    // SYNC0 CyclShift (ns) the bring-up passed to arm_dc_sync (the config's dc_sync0_shift_ns).
    // Lets an offline test assert the config value threads through to the backend's DC arm call
    // (#32 note 4). TEST-ONLY: read after the controller is stopped/joined.
    std::int32_t configured_dc_sync0_shift_ns() const noexcept;
    // #47 test hook: how many times set_state(_, Op) was requested (the no-hammer
    // invariant metric -- the Runner/bring-up must request OP exactly once per start).
    int op_requests() const noexcept {
        return op_requests_;
    }
    // Read back a recorded SDO value (latest write to that object).
    std::vector<std::byte> recorded_sdo(std::uint16_t slave, std::uint16_t index, std::uint8_t sub) const;
    // Ordered log of SDO write keys ((index<<8)|sub) for asserting the remap
    // sub-protocol ordering (configure() test).
    std::vector<std::uint32_t> sdo_log(std::uint16_t slave) const;
    // Make a subsequent sdo_write to object index:sub ABORT (throw SdoError carrying `abort_code`),
    // modelling a drive rejecting an SDO download. Lets the #32 note-14 test assert the error tier:
    // a non-mapping object aborts -> SdoError; a mapping object (0x1C1x/0x16xx/0x1Axx) aborts ->
    // apply_pdo_map re-tags it PdoMappingError. Idempotent; call once per object to arm.
    void set_sdo_write_abort(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, std::uint32_t abort_code = 0x06090011U);

   private:
    struct Slave {
        SimSlaveModel model;
        std::vector<std::byte> output_image;                         // RxPDO (master writes)
        std::vector<std::byte> input_image;                          // TxPDO (master reads)
        std::map<std::uint32_t, std::vector<std::byte>> dictionary;  // recorded SDO writes, key=(index<<8)|sub
        std::vector<std::uint32_t> sdo_write_order;                  // SDO write keys in order (configure() ordering test)
        std::map<std::uint32_t, std::uint32_t> sdo_write_aborts;     // test: key=(index<<8)|sub -> CoE abort code to throw
        EcatState state = EcatState::Init;
        Cia402State device_state = Cia402State::NotReadyToSwitchOn;
        std::uint16_t prev_ctrlword = 0;
        std::int32_t target = 0;
        std::int32_t target_written =
            0;  // #47-P3b: the last 0x607A the master WROTE this cycle (seed/mirror wire value, latched-or-not) -- DA no-lunge probe
        std::int32_t actual = 0;
        bool noise_phase = false;        // #59: encoder-noise square-wave phase (toggles per cycle when model.report_noise>0)
        std::int32_t reported_prev = 0;  // #59: previous REPORTED (jittered) actual -> the jittered 0x606C delta
        bool setpoint_ack = false;       // PP bit12 latch
        // ATOMIC: written by a non-RT test hook (inject_fault/set_fault_code/
        // set_stale_fault_code) while the RT loop reads them in step_device -- the only
        // cross-thread Slave fields. Relaxed is sufficient (independent test signals).
        std::atomic<bool> faulted{false};
        std::atomic<std::uint16_t> fault_code{0};        // 0x603F code reported while faulted (set_fault_code)
        std::atomic<std::uint16_t> stale_fault_code{0};  // forces 0x603F = this REGARDLESS of fault state (flag-gating test)
        std::int32_t profile_velocity = 0;               // last 0x6081 seen in the command image (test visibility; RT-only)
        std::int32_t velocity = 0;                       // per-cycle actual delta (0x606C de-mask; RT-only)
        // #53 PV/quick-stop state (RT-only; read by tests AFTER stop/join):
        std::int32_t pv_velocity = 0;      // current PV/QSA velocity (toy counts/cycle == 0x606C)
        std::int32_t target_velocity = 0;  // last 0x60FF seen in the command image (test visibility)
        std::int32_t velocity_at_qsa_exit =
            0x7fffffff;            // 0x606C the cycle the device left QuickStopActive -> SwitchOnDisabled (proves ramp-then-disable)
        bool entered_qsa = false;  // device ever reached QuickStopActive (proves Quick-Stop, not torque-cut)
        std::atomic<bool> suppress_ack{false};                  // test hook (toggled live during a handshake): never assert bit12
        std::atomic<std::uint32_t> fault_clear_delay{0};        // #18 type-(a) reflect latency (cycles); 0 = instant
        std::atomic<bool> fault_persistent{false};              // #18 type-(b) cause-persists: reset edge ignored
        std::atomic<std::uint32_t> clear_then_refault_hold{0};  // #18 type-(c) momentary-clear hold cycles (0 = off)
        std::uint32_t clear_countdown_ = 0;                     // #18 RT-only: active type-(a) reflect-delay countdown in Fault
        std::uint32_t refault_countdown_ = 0;                   // #18 RT-only: cycles until the type-(c) re-fault fires
        std::atomic<std::uint32_t> fault_reset_edges{0};        // #18 cumulative bit7 0->1 edges seen (no-spin test: RT-write/test-read)
        // #61 (DA): monotonic count of 0x6060 OUTPUT-byte value CHANGES (RT-write/test-read). The
        // unconfirmable-switch STORM gate -- a wrapper that fails to revert switch_intent_ leaves
        // run_mode_switch_ re-arming FOREVER (StopFirst writes the current mode, Settle writes the
        // wanted mode -> the byte oscillates every ~settle-window), so this keeps CLIMBING; with the
        // revert it FREEZES after the single failed attempt. A test polls it twice: stable => no storm.
        std::atomic<std::uint32_t> mode_out_transitions{0};
        std::int8_t prev_mode_out = 0;  // RT-only: last 0x6060 byte written (the transition edge detector)
        bool mode_out_seen = false;     // RT-only: first-write guard (don't count the initial establishment)
        // RUNTIME mode of operation -- set ONLY by the master's 0x6060 SDO write (de-masked
        // from model.mode), so a missing/wrong mode set leaves it None and the motor never
        // moves (mode-0 guard), catching the "forgot to set 0x6060" bug offline.
        Cia402Mode effective_mode = Cia402Mode::None;
        // #47-P3b M56S: pending (not-yet-applied) mode + its remaining latency countdown -- a new 0x6060
        // is applied to effective_mode only after model.mode_switch_latency cycles (the slow-device window).
        Cia402Mode mode_pending = Cia402Mode::None;
        std::uint32_t mode_pending_cycles = 0;
    };

    static void step_device(Slave& s) noexcept;

    // deque, not vector: Slave holds atomics (cross-thread test-hook fields) so it is
    // non-movable, and vector back-insertion compile-time-requires move-insertable
    // (for its reallocation path) even with reserve(). deque grows without moving
    // existing elements, so it stores non-movable Slaves directly. Per-Slave access is
    // still O(1); the inner output_image/input_image vectors stay contiguous.
    std::deque<Slave> slaves_;
    int expected_wkc_ = 0;
    std::uint32_t dc_cycle_ns_ = 0;       // last configure_dc_sync() cycle (0 = never requested)
    std::int32_t dc_sync0_shift_ns_ = 0;  // last arm_dc_sync() SYNC0 CyclShift (config dc_sync0_shift_ns; #32 note 4)
    std::int64_t synthetic_dc_ns_ = 0;    // synthetic DC clock, advanced each exchange() (dc_time())
    bool open_ = false;
    bool short_wkc_once_ = false;
    int op_requests_ =
        0;  // #47: set_state(_, Op) call count (no-hammer metric)                // one-shot (master_test drives it synchronously; RT-only)
    std::atomic<bool> short_wkc_sticky_{false};  // toggled non-RT while the RT loop reads it in exchange() -> atomic
};

}  // namespace ethercat
