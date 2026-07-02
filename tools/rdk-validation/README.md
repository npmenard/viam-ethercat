# RDK validation harness

Implements the campaign in `docs/viam-driven-validation.md` against machine
`ethercat-test` (part `59e04139-…`, addr `ethercat-test-main.03jg17i9wj.viam.cloud`)
with the real A6 on `enp86s0`. Blanket shaft-clear/energize go-ahead applies
(user decision 2026-07-02); the module auto-energizes on every viam-server start.

Python env: `/home/viam/ethercat/.venv-test` (viam-sdk installed).

## Pieces
- `scenario_configs.py` — machine configs: `empty`, `valid` (switchable, #61 derived
  PDO map, #59 default reach tolerance), `wrong`, `wrong_interface`, `dc_validation`,
  `dc_drive` (freerun → AL 0x0027, safe), `dc_drive_sync0` (125 µs SYNC0 lie — WEDGE
  RISK, run once and only if needed).
- `machine_config.py show|set <scenario>|clear` — pushes configs via the app API.
- `server.sh start|stop|status|log` — runs viam-server-static as root (module inherits
  caps); logs under `/home/viam/rdk/logs/`.
- `rdk_tests.py [n…]` — Tests 1–6. Test5-1 is characterize-only (client cancel likely
  never reaches the module). Test6 needs the SDO DoCommands (task #1).
- `cpu_watch.sh [s]` — module %CPU during a run.

## Scenario driving (the "What to test" list)
1. `clear` → start server → confirm no servo → `set valid` (live reconfigure) → Tests 1–6.
2. `clear` → `set wrong` → error in logs (`server.sh log`).
3. `set dc_validation` (config-time reject in logs) AND `set dc_drive` (on-wire drive
   rejection surfaced, no wedge). Both per user decision.
4. `clear` → `set valid` → tests → `set valid` again (reconfigure) → tests still pass.
5. `clear` → `set valid` → tests → `set wrong` → error → `set valid` → tests pass.

Each energized scenario: capture with
`sudo tcpdump -i enp86s0 -w /tmp/rdkval-<scenario>.pcap 'ether proto 0x88a4' -U`
and decode with `tools/bench/ecpcap.py`. Describe any bug found before fixing it
(user rule).
