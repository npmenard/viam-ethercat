# Bug #73 — RDK: reconfigure of an errored resource node orphans the old instance

_Archived 2026-07-03 (user: "we will do it later"). Investigation COMPLETE and proven;
only the upstream filing + the optional module-side safety net remain. Repro artifacts:
`docs/rdk-issue-73/` (trivial Go module + local config + log excerpt)._

## Symptom (found by campaign scenario 5)
With a healthy modular resource running, push a config that fails module `ValidateConfig`,
then push a corrected config → RDK loops forever on
`Attempted to add resource that already existed` (both C++ and Go SDKs); the servo is
unusable until the module process restarts. **Safety hazard: the orphaned old instance
never receives `Close()` — a hardware axis stays energized while unreachable by clients.**

## Root cause (rdk @ 04ee9978e, line-verified + reproduced with a Go module)
1. Validate fails → `completeConfig` calls `gNode.LogAndSetLastError(...)` and returns
   (`robot/impl/resource_manager.go:815-822`) — no Close, no RemoveResource; the node keeps
   its live `current` resource in `lastErr` state.
2. Recovery push → `processResource` → `closeAndUnsetResource` (`:612-621`) calls
   `gNode.Resource()`, which returns `lastErr` for an errored node (`resource/graph_node.go:168-180`)
   → early-return **skips** `closeResource` (the RemoveResource sender, `:598`) **and**
   `UnsetResource()` → continues to `newResource` → module `AddResource` → collision → retry forever.
3. Each retry constructs + closes a fresh throwaway instance; the ORIGINAL orphan is never closed
   (proven via the repro module's own Close logging).

## Introducing commit (regression, not an original defect)
**`2525d53cba1235536b2909811f1021bc603f0618` — "RSDK-5838 - Remove resource from node if
missing dependencies (#3386)", 2024-01-02, first shipped v0.18.0.** Before it, the rebuild
path used `gNode.UnsafeResource()` (the documented errored-node reconfiguration accessor,
`graph_node.go:210-213`) and correctly Closed + RemoveResource'd the old instance (correct
since PR #2225, 2023-04). #3386 swapped in `Resource()`, completing the broken pair.
Carried ~2.5 years / 168 minor releases (v0.18.0 → v0.132.0-era).

## Suggested upstream fix
`closeAndUnsetResource` (and the rebuild path in `processResource`) should use
`gNode.UnsafeResource()` so an errored-but-current resource is Closed + RemoveResource'd +
unset before the rebuild's Add. (Alternatively/additionally close+remove at the
Validate-failure branch itself.)

## Reproduction (~2 min, no hardware)
1. `cd docs/rdk-issue-73 && GOFLAGS=-mod=mod go build -o repro73-module .`
   (go.mod `replace`s go.viam.com/rdk with the local checkout).
2. `viam-server-static -config viam-config.json` (local config, no cloud; binds localhost).
3. Wait for the module's `REPRO73_CONSTRUCT ... LIVE` log; edit the config: `bad: true`
   (Validate fails — note NO Close on the instance); edit back to `bad: false` →
   observe the `already exists` loop (`REPRO_EXCERPT.txt` shows a captured run).
4. Grep the original instance pointer for a Close line — none until process shutdown.

## Remaining work (archived)
- File the upstream issue at viamrobotics/rdk (drafted text = the sections above; dup-check first) — or fix directly (one-accessor change + a regression test).
- Optional module-side safety net: a per-Name registry so a colliding recovery-Add
  de-energizes the orphaned instance (fixes the safety hazard; cannot fix the brick).
- Operational workaround until then: after a valid→invalid→valid cycle, restart the module
  process; the axis holds torque while bricked.
