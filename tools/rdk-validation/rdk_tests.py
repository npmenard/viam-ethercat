"""RDK validation test sequence (Tests 1-6 of docs/viam-driven-validation.md).

Runs against the live machine (ethercat-test-main.03jg17i9wj.viam.cloud) with the
REAL A6 energized. Shaft-clear blanket go-ahead applies to the whole campaign.

Usage:
    python rdk_tests.py            # full sequence 1..6
    python rdk_tests.py 2 3        # only Test2 and Test3

Test5-1 (asyncio cancel) is a CHARACTERIZE step per user decision: we record what
actually happens (the cancel likely never reaches the module) instead of pass/fail.
"""

import asyncio
import sys
import time

from viam.robot.client import RobotClient
from viam.components.motor import Motor

ADDRESS = "ethercat-test-main.03jg17i9wj.viam.cloud"
API_KEY = "g6o8sr4zh49yefpuhvpn909vyc0e0ggi"
API_KEY_ID = "403431c2-0187-4759-a0db-ab6fc43bd1af"

POS_TOL = 0.1  # revs; assertion tolerance for reached-position checks (drive tol is ~0.0014 rev)

RESULTS: list[tuple[str, str, str]] = []  # (step, PASS/FAIL/INFO, detail)


def record(step: str, ok: bool | None, detail: str) -> None:
    verdict = "INFO" if ok is None else ("PASS" if ok else "FAIL")
    RESULTS.append((step, verdict, detail))
    print(f"  [{verdict}] {step}: {detail}", flush=True)


async def connect() -> RobotClient:
    opts = RobotClient.Options.with_api_key(api_key=API_KEY, api_key_id=API_KEY_ID)
    return await RobotClient.at_address(ADDRESS, opts)


async def settle_position(servo: Motor, seconds: float = 2.0, interval: float = 0.25) -> tuple[float, float]:
    """Sample position over `seconds`; return (final_position, max_abs_delta_between_samples)."""
    samples = [await servo.get_position()]
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        await asyncio.sleep(interval)
        samples.append(await servo.get_position())
    deltas = [abs(b - a) for a, b in zip(samples, samples[1:])]
    return samples[-1], (max(deltas) if deltas else 0.0)


async def test1_connect(machine: RobotClient) -> bool:
    print("== Test1: Connect ==")
    # poll: after a live config push, module spawn + A6 DC bring-up takes seconds
    deadline = time.monotonic() + 30
    while True:
        names = [n.name for n in machine.resource_names]
        if "servo" in names or time.monotonic() >= deadline:
            break
        await asyncio.sleep(2)
        await machine.refresh()
    print(f"  resources: {names}")
    ok = "servo" in names
    record("T1 servo in resources", ok, f"resource names: {names}")
    return ok


async def test2_setrpm(servo: Motor) -> None:
    print("== Test2: SetRPM ==")
    p0 = await servo.get_position()
    await servo.set_rpm(1000)
    await asyncio.sleep(1.5)
    p1 = await servo.get_position()
    record("T2.1 set_rpm(1000) position increases", p1 > p0 + 1.0, f"{p0:.2f} -> {p1:.2f} revs in 1.5s")

    await servo.set_rpm(-1000)
    await asyncio.sleep(1.0)
    p2 = await servo.get_position()
    record("T2.2 set_rpm(-1000) position decreases", p2 < p1 - 1.0, f"{p1:.2f} -> {p2:.2f} revs in 1.0s")

    await servo.set_rpm(0)
    await asyncio.sleep(1.0)  # decel
    p3, drift = await settle_position(servo, 1.5)
    moving = await servo.is_moving()
    record("T2.3 set_rpm(0) stops", drift < 0.05 and not moving,
           f"pos {p3:.2f}, max inter-sample drift {drift:.4f} revs, is_moving={moving}")


async def test3_gofor(servo: Motor) -> None:
    print("== Test3: GoFor ==")
    cases = [  # (rpm, revolutions, expected signed delta)
        (1200, 60, +60), (1200, -60, -60), (-1200, -60, +60), (1200, -60, -60),
    ]
    for i, (rpm, revs, want) in enumerate(cases, 1):
        start = await servo.get_position()
        t0 = time.monotonic()
        await servo.go_for(rpm=rpm, revolutions=revs)
        dt = time.monotonic() - t0
        end = await servo.get_position()
        got = end - start
        record(f"T3.{i} go_for({rpm},{revs}) -> {want:+d} revs", abs(got - want) < POS_TOL,
               f"delta {got:+.3f} revs (want {want:+d}) in {dt:.1f}s")


async def test4_goto(servo: Motor) -> None:
    print("== Test4: GoTo ==")
    for i, target in enumerate([0, -100, 100, 0], 1):
        t0 = time.monotonic()
        await servo.go_to(rpm=2000, position_revolutions=target)
        dt = time.monotonic() - t0
        pos = await servo.get_position()
        record(f"T4.{i} go_to(2000,{target})", abs(pos - target) < POS_TOL,
               f"pos {pos:+.3f} (want {target:+d}) in {dt:.1f}s")


async def test5_cancel_stop_reject(servo: Motor) -> None:
    print("== Test5: cancellation / stop / rejection ==")
    # 5.1 asyncio cancel -- CHARACTERIZE (user decision): record what actually happens.
    task = asyncio.ensure_future(servo.go_to(rpm=2000, position_revolutions=1000))
    await asyncio.sleep(6)
    task.cancel()
    try:
        await task
        cancel_outcome = "go_to returned normally despite cancel"
    except asyncio.CancelledError:
        cancel_outcome = "client task cancelled"
    except Exception as e:  # noqa: BLE001 - characterizing
        cancel_outcome = f"go_to raised {type(e).__name__}: {e}"
    p_a = await servo.get_position()
    await asyncio.sleep(3)
    p_b = await servo.get_position()
    moving = await servo.is_moving()
    still_moving = abs(p_b - p_a) > 1.0 or moving
    record("T5.1 asyncio-cancel behavior", None,
           f"{cancel_outcome}; 3s after cancel: moved {p_b - p_a:+.2f} revs, is_moving={moving} "
           f"-> motor {'KEPT MOVING' if still_moving else 'stopped'}")
    if still_moving:
        await servo.stop()
        await asyncio.sleep(1)

    # 5.2 recover to 0
    await servo.go_to(rpm=2000, position_revolutions=0)
    pos = await servo.get_position()
    record("T5.2 go_to(2000,0)", abs(pos) < POS_TOL, f"pos {pos:+.3f}")

    # 5.3 stop() during a blocking go_to -> the go_to call must FAIL with an error.
    task = asyncio.ensure_future(servo.go_to(rpm=2000, position_revolutions=-1000))
    await asyncio.sleep(6)
    await servo.stop()
    try:
        await task
        record("T5.3 stop() fails the in-flight go_to", False, "go_to returned normally -- expected an error")
    except Exception as e:  # noqa: BLE001
        record("T5.3 stop() fails the in-flight go_to", True, f"go_to raised {type(e).__name__}: {e}")
    await asyncio.sleep(1)

    # 5.4 recover to 0
    await servo.go_to(rpm=2000, position_revolutions=0)
    pos = await servo.get_position()
    record("T5.4 go_to(2000,0)", abs(pos) < POS_TOL, f"pos {pos:+.3f}")

    # 5.5 set_rpm during a blocking go_to -> must be REJECTED; then stop.
    task = asyncio.ensure_future(servo.go_to(rpm=2000, position_revolutions=1000))
    await asyncio.sleep(2)
    try:
        await servo.set_rpm(1000)
        record("T5.5 set_rpm rejected during go_to", False, "set_rpm succeeded -- expected rejection")
    except Exception as e:  # noqa: BLE001
        record("T5.5 set_rpm rejected during go_to", True, f"set_rpm raised {type(e).__name__}: {e}")
    await servo.stop()
    try:
        await task
    except Exception as e:  # noqa: BLE001
        print(f"  (in-flight go_to ended with: {type(e).__name__}: {e})")
    await asyncio.sleep(1)

    # 5.6 recover to 0
    await servo.go_to(rpm=2000, position_revolutions=0)
    pos = await servo.get_position()
    record("T5.6 go_to(2000,0)", abs(pos) < POS_TOL, f"pos {pos:+.3f}")


async def test6_docommand(servo: Motor) -> None:
    print("== Test6: DoCommand (SDO reads mid-move) ==")
    task = asyncio.ensure_future(servo.go_to(rpm=2000, position_revolutions=2000))
    await asyncio.sleep(2)
    # success key per command; a failed SDO read returns {"<key>_error": ...} which must FAIL
    expect = {
        "get_motor_voltage": "voltage_volts",
        "get_motor_current_actual_value": "current_amps",
        "get_motor_drive_modes": "drive_modes",
    }
    for cmd, key in expect.items():
        try:
            resp = dict(await servo.do_command({cmd: True}))
            errs = [k for k in resp if k.endswith("_error")]
            record(f"T6 {cmd}", key in resp and not errs, f"response: {resp}")
        except Exception as e:  # noqa: BLE001
            record(f"T6 {cmd}", False, f"raised {type(e).__name__}: {e}")
    moving = await servo.is_moving()
    record("T6 motor still moving after DoCommands", moving, f"is_moving={moving}")
    await servo.stop()
    try:
        await task
    except Exception as e:  # noqa: BLE001
        print(f"  (in-flight go_to ended with: {type(e).__name__}: {e})")
    await asyncio.sleep(1)
    await servo.go_to(rpm=2000, position_revolutions=0)
    pos = await servo.get_position()
    record("T6 recover go_to(2000,0)", abs(pos) < POS_TOL, f"pos {pos:+.3f}")


async def main() -> int:
    wanted = {int(a) for a in sys.argv[1:]} or {1, 2, 3, 4, 5, 6}
    async with await connect() as machine:
        if not await test1_connect(machine):
            print("Test1 failed -- aborting (per test plan).")
            return 1
        if wanted - {1}:
            servo = Motor.from_robot(machine, "servo")
            if 2 in wanted:
                await test2_setrpm(servo)
            if 3 in wanted:
                await test3_gofor(servo)
            if 4 in wanted:
                await test4_goto(servo)
            if 5 in wanted:
                await test5_cancel_stop_reject(servo)
            if 6 in wanted:
                await test6_docommand(servo)

    print("\n==== SUMMARY ====")
    fails = 0
    for step, verdict, detail in RESULTS:
        print(f"[{verdict}] {step} -- {detail}")
        fails += verdict == "FAIL"
    print(f"{len(RESULTS)} steps, {fails} failures")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
