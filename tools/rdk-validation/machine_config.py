"""Machine-config management for the RDK validation campaign (docs/viam-driven-validation.md).

Pushes scenario configs to the ethercat-test machine part via the Viam app API
(the CLI's add-resource can't carry attribute JSON). Credentials are the machine
API key from /home/viam/rdk/viam.json.

Usage:
    python machine_config.py show
    python machine_config.py set <scenario>     # scenario from scenario_configs.SCENARIOS
    python machine_config.py clear              # empty config
"""

import asyncio
import json
import sys

from viam.app.viam_client import ViamClient
from viam.rpc.dial import DialOptions

import scenario_configs

PART_ID = "59e04139-bc28-498a-b55a-485f8ea4789c"
API_KEY = "g6o8sr4zh49yefpuhvpn909vyc0e0ggi"
API_KEY_ID = "403431c2-0187-4759-a0db-ab6fc43bd1af"


async def _client() -> ViamClient:
    return await ViamClient.create_from_dial_options(DialOptions.with_api_key(API_KEY, API_KEY_ID))


async def show() -> None:
    client = await _client()
    try:
        part = await client.app_client.get_robot_part(PART_ID)
        print(f"part: {part.name} ({part.id})")
        print(json.dumps(part.robot_config, indent=2))
    finally:
        client.close()


async def set_config(config: dict, label: str) -> None:
    client = await _client()
    try:
        part = await client.app_client.get_robot_part(PART_ID)
        await client.app_client.update_robot_part(PART_ID, part.name, config)
        print(f"config '{label}' pushed to part {part.name} ({PART_ID})")
    finally:
        client.close()


def main() -> None:
    if len(sys.argv) < 2 or sys.argv[1] not in ("show", "set", "clear"):
        print(__doc__)
        print("scenarios:", ", ".join(scenario_configs.SCENARIOS))
        sys.exit(2)
    cmd = sys.argv[1]
    if cmd == "show":
        asyncio.run(show())
    elif cmd == "clear":
        asyncio.run(set_config(scenario_configs.empty(), "empty"))
    else:
        name = sys.argv[2]
        cfg = scenario_configs.build(name)
        asyncio.run(set_config(cfg, name))


if __name__ == "__main__":
    main()
