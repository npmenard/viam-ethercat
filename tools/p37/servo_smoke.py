#!/usr/bin/env python3
"""#37 module-on-real-A6 smoke/energized driver (Viam RDK path).

Connects to a LOCAL viam-server running the viam:ethercat:servo module against the real A6
(etc/a6-robot.example.json), and exercises the rdk:component:motor API.

DEFAULT = READ-ONLY SMOKE (no motion): connect -> get_properties -> is_powered -> get_position.
--energize adds motion: reset_zero -> go_to(rpm, +revs) -> poll is_moving/position to reached -> stop.
Run tcpdump alongside (see the #37 recipe) to prove no-coast on the wire.

DEP (LONG-POLE): the Viam Python SDK is NOT installed on this box. Install first:
    pip install --user viam-sdk
(Alternatively drive the same sequence from the C++ SDK, which IS present, or `viam` CLI.)

ENV:
    VIAM_ADDRESS   robot part address (default localhost:8080 for a local-only viam-server)
    VIAM_API_KEY / VIAM_API_KEY_ID   optional, only if the part is cloud-auth'd
    MOTOR_NAME     component name (default 'servo', matches etc/a6-robot.example.json)
"""
import argparse, asyncio, os, sys

def _opts():
    from viam.robot.client import RobotClient
    key, kid = os.getenv("VIAM_API_KEY"), os.getenv("VIAM_API_KEY_ID")
    if key and kid:
        return RobotClient.Options.with_api_key(api_key=key, api_key_id=kid)
    # local-only viam-server (no cloud auth): insecure gRPC, no WebRTC signaling.
    from viam.rpc.dial import DialOptions
    return RobotClient.Options(dial_options=DialOptions(insecure=True, disable_webrtc=True))

async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--energize", action="store_true", help="ENERGIZED: move the shaft (default: read-only smoke)")
    ap.add_argument("--rpm", type=float, default=60.0)
    ap.add_argument("--revs", type=float, default=0.25)
    ap.add_argument("--timeout", type=float, default=15.0, help="seconds to wait for the move to reach")
    args = ap.parse_args()

    try:
        from viam.robot.client import RobotClient
        from viam.components.motor import Motor
    except ImportError:
        print("FATAL: viam-sdk not installed -> `pip install --user viam-sdk`", file=sys.stderr)
        return 3

    addr = os.getenv("VIAM_ADDRESS", "localhost:8080")
    name = os.getenv("MOTOR_NAME", "servo")
    print(f"[smoke] connecting to {addr}, motor '{name}' (energize={args.energize})")
    robot = await RobotClient.at_address(addr, _opts())
    try:
        motor = Motor.from_robot(robot, name)

        props = await motor.get_properties()
        print(f"[smoke] properties: position_reporting={props.position_reporting}")
        powered, pct = await motor.is_powered()
        pos = await motor.get_position()
        print(f"[smoke] is_powered={powered} ({pct:.2f})  position={pos:.4f} rev")
        # The #57 mode-echo gate: a healthy bring-up energizes; a silent mode-ignore leaves powered=False
        # (last_error names the mismatch). Read-only smoke ASSERTS bring-up reached OE:
        if not powered:
            print("[smoke] NOT powered -> bring-up did not reach OperationEnabled "
                  "(check viam-server logs / DoCommand last_error; #57 gate may have refused a wrong mode).",
                  file=sys.stderr)
            return 2
        print("[smoke] READ-ONLY smoke PASSED (energized to OE, position readable).")

        if args.energize:
            print(f"[energize] reset_zero -> go_to({args.rpm} rpm, +{args.revs} rev)")
            await motor.reset_zero_position(0.0)
            await motor.go_to(args.rpm, args.revs)   # PP absolute; blocks server-side until reached/timeout
            # poll the client view to corroborate
            deadline = asyncio.get_event_loop().time() + args.timeout
            reached = False
            while asyncio.get_event_loop().time() < deadline:
                moving = await motor.is_moving()
                p = await motor.get_position()
                print(f"[energize] is_moving={moving} position={p:.4f}")
                if (not moving) and abs(p - args.revs) < 0.02:
                    reached = True; break
                await asyncio.sleep(0.2)
            await motor.stop()  # R1 MOTION-stop = Halt (energized hold)
            final = await motor.get_position()
            still_on, _ = await motor.is_powered()
            print(f"[energize] after stop: position={final:.4f} rev, is_powered={still_on} "
                  f"(R1: Halt stays ENERGIZED). reached={reached}")
            return 0 if reached else 2
        return 0
    finally:
        await robot.close()

if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
