# Spec #21 — Thin ec_sample-equivalent test program on our Master API

**Status:** ready after #20 (the bring-up it wraps). Architect-owned.
**Goal:** prove "**our** library brings the A6 to OPERATIONAL in DC (WKC 3/3, live PDO)" via a **~15-line** program with **ZERO raw EtherCAT logic** — no `ecx_*`, no AL states, no DC registers, no SOEM. The driver does the heavy lifting; the program just constructs config → starts → confirms OP → reads feedback → stops.

---

## 0. The litmus test

The test program must contain **no EtherCAT vocabulary**: no `ecx_`/`ec_`, no `SAFE_OP`/`SYNC0`/`0x1C32`/`0x6040`, no phase-lock, no WKC arithmetic. If it does, the API isn't thin enough — push that logic down into the driver. The program reads like "bring this drive up and tell me its position," not like ec_sample.c.

---

## 1. Recommended approach: reuse `ServoController` + 2 thin accessors (fastest to first light)

`ServoController` ALREADY owns everything the test needs: `start()` (configure + single RT thread), the #20 DC bring-up to OP (folded into `run_rt_loop`), the atomic startup handshake (signals "reached OP"), published feedback (`ControllerState`), `last_error()` (#16), `stop()` (clean teardown: SYNC0 off → join → close). So the A6 reaching OP via `ServoController::start()` **IS** the ec_sample-equivalent proof. We need only **two thin, master_-free accessors** on top:

- **`bool is_operational() const noexcept`** — EtherCAT OPERATIONAL reached + DC-synced + WKC holding. This is the **bus-level** signal (the #20 bring-up endpoint), **distinct from `is_powered()`** (which is CiA402 OperationEnabled — motor energized). The #20 bring-up already publishes a "reached OP" atom (the (b)-design StartupState/ReachedOp); `is_operational()` reads it + `rt_alive()`. Lock-free, no `master_`.
- **`FeedbackSnapshot feedback() const noexcept`** — a lock-free snapshot of the published feedback (see §2). Reuses the `ControllerState` atoms; add the raw statusword if not already published.

**Why reuse over a new `BusRunner` type:** it's the fast path to first light (team-lead's priority), zero new RT-thread/teardown machinery (all of `ServoController`'s hard-won lifecycle — jthread join-before-destroy, the atomic handoff, reconfigure-safety — is reused), and the A6 *is* a motor so there's no conceptual mismatch. The test simply doesn't call the motion API.

**Alternative (note, not recommended now):** factor a `BusRunner` base (RT-thread + #20 bring-up + published feedback) that `ServoController` extends and the test uses bare — cleaner separation if a non-motor slave ever needs bring-up without CiA402. Defer until there's a second consumer; #20 §9 already keeps the bring-up loop CiA402-independent so this factoring stays cheap later. For #21, the 2 accessors on `ServoController` ship first light now.

---

## 2. `FeedbackSnapshot` (the read surface)

A small value struct, all fields from the published `ControllerState` (lock-free, master_-free read; one consistent snapshot):
```cpp
struct FeedbackSnapshot {
    bool          operational;     // EtherCAT OP + DC-synced (== is_operational())
    std::uint16_t statusword;      // 0x6041 raw (decode externally if desired)  [ADD to ControllerState if absent]
    std::int32_t  position_counts; // 0x6064 actual (raw device counts)
    std::int32_t  velocity;        // 0x606C / estimate (device units, #16)
    std::uint16_t drive_fault_code;// 0x603F (0 = none; 0x8700 = Er74.1 no-sync, 0x6320 = Er74.0)
    std::int32_t  wkc;             // last working counter (== expected when healthy)
};
FeedbackSnapshot feedback() const noexcept;
```
- `position_revs()`/`velocity_counts()` already exist; `feedback()` bundles them + the raw statusword + fault code + WKC into one snapshot so the test reads everything in a single call. Raw counts (not revs) so the bus-bring-up test is unit-agnostic; the module layer converts.
- The statusword + WKC may need publishing into `ControllerState` (a `std::atomic<std::uint16_t> statusword` + reuse `fault_wkc`/a `last_wkc` atom). Minimal additions.

---

## 3. The test program (~15 lines, zero EtherCAT logic)

`tools/bench/a6_run.cpp` (or similar), built in the bench config:
```cpp
int main(int argc, char** argv) {
    const std::string ifname = argc > 1 ? argv[1] : "enp86s0";
    ServoConfig cfg = load_servo_config(argc > 2 ? argv[2] : "etc/a6-hardware.example.json");
    cfg.ifname = ifname;

    ServoController drive{cfg};                         // production ctor (SoemBackend)
    drive.start();                                      // open + configure(DC) + RT thread + run to OP; throws InitError w/ AL detail on failure

    if (!drive.is_operational()) { std::cerr << drive.last_error() << '\n'; return 1; }
    std::cout << "OPERATIONAL (DC). streaming feedback:\n";
    for (int i = 0; i < 500; ++i) {                     // ~5 s of live PDO
        const auto fb = drive.feedback();
        std::cout << "  sw=0x" << std::hex << fb.statusword << std::dec
                  << " pos=" << fb.position_counts << " vel=" << fb.velocity
                  << " wkc=" << fb.wkc << (fb.drive_fault_code ? " FAULT" : "") << '\n';
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    drive.stop();                                       // SYNC0 off -> join -> close
    return 0;
}
```
Every line is config/lifecycle/feedback. No `ecx_*`, no state machine, no DC math. (Optional bonus, still thin: `drive.enable(); drive.go_to(rpm, revs);` to prove a live move — but the core ec_sample-equivalent is reach-OP + stream feedback.)

---

## 4. What the driver owns (so the program doesn't)

Everything ec_sample.c does by hand, our API hides inside `start()` + the RT loop (#20):
- NIC open + enumerate + the PDO-remap SDO sub-protocol + `config_map_group` + `configdc` + reach SAFE-OP (configure()).
- The single RT thread; phase-locked PD from cycle 0; stock `ecx_dcsync0` arm in-loop; the PD-health + no-Er74.1 OP-gate; the gapless SAFE-OP→OP; vendor fault-reset (`0x2031:01`, config-data).
- Teardown: SYNC0-off-before-close, join-before-destroy.
- Failure legibility: `start()` throws `InitError` with the AL-status detail; `last_error()` composes the #16 tiers (drive fault 0x8700/0x6320 + WKC). So even the *failure* path needs no raw EtherCAT in the test — the driver reports causes in human text.

---

## 5. Config loading (`load_servo_config`)

The test needs a config without hand-building it. Reuse the module's attrs→`ServoConfig` parser path (#12) but from a JSON file directly (the bench has no Viam SDK ResourceConfig). Either a thin `ServoConfig load_servo_config(path)` (parse `etc/a6-hardware.example.json` — the PDO maps, cycle, `use_distributed_clocks`, the `0x2031:01` fault-reset descriptor, fault-code labels) or reuse #12's parser against a raw ProtoStruct loaded from JSON. Keep it config-data; no A6 constants in the test.

---

## 6. Acceptance + sequence

- **Acceptance:** on the bench, `sudo ./build-hw/a6_run enp86s0 etc/a6-hardware.example.json` → "OPERATIONAL (DC)" + streaming feedback with `wkc=3`, no fault — the ec_sample-equivalent on our stack. Offline: an analogous SimBackend-backed unit (construct → start → is_operational → feedback → stop) so the thin API is CI-covered too.
- **Litmus (§0) enforced in review:** grep the test program for `ecx_`/`ec_`/`SYNC`/`0x1C`/`0x60` → must be empty.
- **Sequence:** after #20 lands the bring-up, cpp-expert adds the 2 accessors (`is_operational`, `feedback`) + any `ControllerState` field (statusword/wkc) + `load_servo_config` + the `a6_run.cpp` program. Review: me (the API surface is genuinely thin + reuses the lifecycle correctly; `is_operational` vs `is_powered` distinction) + DA (the accessors are lock-free/master_-free/snapshot-consistent; the test has zero EtherCAT logic). Then bench-validate to first light.

---

## 7. Note on the bigger picture

This #21 program is the **first-light proof on our own driver** — once it streams `wkc=3` OPERATIONAL feedback, the SOEM-v2 + DC-fold stack is validated end-to-end on our API, and the ServoController motion path (enable → go_to) is the immediate next bench step on the same foundation. The thin API (`start` → `is_operational` → `feedback` → `stop`) is also the shape a downstream integrator would use, so it doubles as the API-ergonomics check.
