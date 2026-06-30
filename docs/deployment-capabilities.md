# Deployment: Granting Linux Capabilities to the EtherCAT Servo Module

The servo module needs a few privileged kernel features that an ordinary
unprivileged process does not have. This document explains **what** it needs,
**why**, and **how** to grant it on the three deployment shapes we support:
a `systemd`-managed `viam-server`/`viam-agent` (recommended), a bare binary via
`setcap`, and Docker.

> TL;DR — the recommended path is a `systemd` **ambient-capabilities drop-in** on
> the `viam-server` (or `viam-agent`) unit. It survives module re-deploys, needs
> no root, and is inherited by the module process automatically. Jump to
> [1. systemd ambient capabilities (recommended)](#1-systemd-ambient-capabilities-recommended).

---

## What the module needs and why

| Capability | Used for | Failure mode without it |
|---|---|---|
| `CAP_NET_RAW` | SOEM opens a **raw packet socket** (`AF_PACKET`) on the EtherCAT NIC. | `ec_init(ifname)` / NIC open fails; the module hard-errors at init. |
| `CAP_NET_ADMIN` | Low-level NIC control alongside the raw socket (promisc/interface state). | Raw EtherCAT traffic may not flow reliably. |
| `CAP_SYS_NICE` | Set the RT thread to `SCHED_FIFO` real-time priority. | `pthread_setschedparam(SCHED_FIFO, …)` returns `EPERM`. With `require_realtime=true` (default) the module errors out; with `false` it falls back to `SCHED_OTHER` (best-effort, non-deterministic). |
| `CAP_IPC_LOCK` | `mlockall(MCL_CURRENT \| MCL_FUTURE)` to pin pages and avoid page-fault jitter in the RT loop. | `mlockall` returns `EPERM`; the loop is subject to paging jitter. |

Two `rlimit`s matter alongside the caps:

- **`RLIMIT_RTPRIO`** (`ulimit -r`) — the ceiling on real-time priority the
  process may request. Must be ≥ the module's configured `rt_priority`
  (default `80`). `CAP_SYS_NICE` lets you exceed the rlimit, but setting a
  generous `LimitRTPRIO=99` is the clean way.
- **`RLIMIT_MEMLOCK`** (`ulimit -l`) — the amount of memory that may be locked.
  `mlockall` needs this high (or unlimited) unless `CAP_IPC_LOCK` is granted.
  Set both for belt-and-suspenders.

### Why capabilities are process-level and inherited across `exec`

`viam-server` launches each module as a **child process** (`fork` + `exec` of the
module entry point). Linux capabilities and rlimits are **per-process**, not
per-thread: you cannot grant `CAP_SYS_NICE` to "just the RT thread." Every thread
the module spawns inherits the process's capability set, so granting the caps to
the module process is exactly what's needed for the RT thread.

The subtlety is **inheritance across `exec`**. A normal capability set is dropped
when a process `exec`s a new program unless one of two things carries it through:

1. **Ambient capabilities** — caps placed in the *ambient* set survive `exec` and
   are raised into the new program's permitted+effective sets automatically. This
   is what the systemd `AmbientCapabilities=` drop-in configures, and it is why a
   cap granted to `viam-server` flows down into the module child. **This is the
   recommended mechanism.**
2. **File capabilities** — caps stored as an xattr on the *binary* (`setcap`).
   These attach to the specific executable, independent of the parent. This is the
   `setcap` fallback below.

Because the module is `exec`'d by `viam-server`, granting ambient caps to the
`viam-server`/`viam-agent` unit is the most robust option: it does not depend on
the module tarball, survives every re-deploy, and needs no per-binary step.

---

## 1. systemd ambient capabilities (recommended)

Add a drop-in to whichever unit actually launches the module on your target —
`viam-server` for a manually-installed server, or `viam-agent` if you run the
agent (the agent then launches `viam-server`, and caps flow down the whole chain).

```bash
# Pick the right unit for your install: viam-server OR viam-agent
sudo systemctl edit viam-server
```

In the editor that opens, add:

```ini
[Service]
# Capabilities granted to the process AND raised into the ambient set so they
# survive exec into the module child process.
AmbientCapabilities=CAP_NET_RAW CAP_NET_ADMIN CAP_SYS_NICE CAP_IPC_LOCK
CapabilityBoundingSet=CAP_NET_RAW CAP_NET_ADMIN CAP_SYS_NICE CAP_IPC_LOCK

# Real-time scheduling + memory locking limits for the RT thread.
LimitRTPRIO=99
LimitMEMLOCK=infinity
```

Then reload and restart:

```bash
sudo systemctl daemon-reload
sudo systemctl restart viam-server      # or: viam-agent
```

Notes:

- `CapabilityBoundingSet` is a ceiling; if your distro's default bounding set
  already drops one of these, naming them here ensures they are available to be
  raised. If you want to *only* allow these and nothing else, this line restricts
  the unit to exactly that set.
- If the unit runs as a non-root `User=`, ambient capabilities still work — that
  is the point of the ambient set. You do **not** need to run `viam-server` as
  root.
- The drop-in lives at
  `/etc/systemd/system/viam-server.service.d/override.conf` (or
  `…/viam-agent.service.d/…`) and is **independent of the module tarball**, so it
  is not lost on module re-deploy/upgrade.

> Open item for your target: confirm the exact unit name and `User=` that
> `viam-server` runs under (`systemctl status viam-server` /
> `systemctl cat viam-server`) so the drop-in lands on the right process.

---

## 2. `setcap` on the module binary (fallback)

If you cannot edit the launcher unit, grant file capabilities directly to the
module's entry-point executable:

```bash
# Path is the module entrypoint that viam-server execs (the unpacked binary).
sudo setcap 'cap_net_raw,cap_net_admin,cap_sys_nice,cap_ipc_lock+ep' /path/to/module-entrypoint
```

Verify:

```bash
getcap /path/to/module-entrypoint
# /path/to/module-entrypoint cap_net_raw,cap_net_admin,cap_sys_nice,cap_ipc_lock=ep
```

Caveats — read before relying on this:

- **Lost on every re-extract.** File caps are an xattr on the binary. Viam
  unpacks `module.tar.gz` fresh on each deploy/upgrade, producing a *new* file
  with **no** xattr. You must re-run `setcap` after every module update (e.g. via
  a post-deploy hook), which is fragile. The systemd approach does not have this
  problem.
- **Filesystem must support and honor xattrs.** The module must live on an
  xattr-capable filesystem (ext4/xfs/btrfs — `tmpfs` and many overlay/`tmpfs`
  module caches do not persist file caps reliably) mounted **without `nosuid`**.
  A `nosuid` mount silently strips file capabilities at `exec`.
- **`setcap` does not raise `RLIMIT_RTPRIO`/`RLIMIT_MEMLOCK`.** With
  `CAP_SYS_NICE`/`CAP_IPC_LOCK` present the limits are effectively bypassed, but
  if you hit limit-related `EPERM` add to `/etc/security/limits.d/` for the
  running user:
  ```
  @viam   -   rtprio    99
  @viam   -   memlock   unlimited
  ```

---

## 3. Docker

When the module (or a dev/CI harness) runs in a container, grant the caps and
limits on the `docker run` line, and give the container a real NIC via host
networking:

```bash
docker run \
  --cap-add=NET_RAW \
  --cap-add=NET_ADMIN \
  --cap-add=SYS_NICE \
  --cap-add=IPC_LOCK \
  --ulimit rtprio=99 \
  --ulimit memlock=-1 \
  --network host \
  <image>
```

Notes:

- `--network host` gives the container direct access to the host's EtherCAT NIC
  by name (e.g. `eth1`). Bridged networking will not expose the raw NIC the
  master needs.
- `--ulimit memlock=-1` means unlimited; `--ulimit rtprio=99` permits the RT
  priority.
- Equivalent `docker-compose`:
  ```yaml
  services:
    servo:
      image: <image>
      cap_add: [NET_RAW, NET_ADMIN, SYS_NICE, IPC_LOCK]
      network_mode: host
      ulimits:
        rtprio: 99
        memlock: -1
  ```
- Avoid `--privileged` — the four `--cap-add` flags are the least-privilege
  equivalent for our needs.

---

## Host requirements (independent of the cap mechanism)

- **Dedicated EtherCAT NIC.** SOEM keeps a NIC in (effectively) raw/promiscuous
  use and EtherCAT is not IP traffic. Use a **separate physical NIC** for the bus
  (`ifname` in the module config, e.g. `eth1`) — never the interface carrying the
  host's management/IP traffic. SOEM also has process-global state: **one master
  / one NIC per process.**
- **PREEMPT_RT kernel.** For deterministic cycle timing the host should run a
  `PREEMPT_RT` (or at least `PREEMPT`) kernel. On a non-RT kernel, RT setup may
  still succeed but jitter will be higher; with `require_realtime=true` (default)
  the module refuses to run if it cannot obtain RT scheduling. Set
  `require_realtime=false` only for dev/CI on non-RT machines.
  Check: `uname -v` should contain `PREEMPT_RT` (or `PREEMPT RT`).

---

## C++ runtime ABI floor on the robot (libstdc++ + glibc)

The module is built in a Docker image (`ubuntu:noble`, GCC 13) and shipped as
`module.tar.gz` — but it **runs on the robot's machine**, dynamically linked
against the robot's `libstdc++.so.6`. That imposes a minimum runtime-library
version on the deployment target.

- **The floor: the robot needs `libstdc++.so.6` providing `GLIBCXX_3.4.32` or
  newer** (the GCC-13 / Ubuntu-24.04 "noble" ABI). A robot with an older
  `libstdc++` fails to **load** the module with
  `version 'GLIBCXX_3.4.32' not found`. This is a raise from the previous
  `ubuntu:jammy` (GCC 11) build, whose floor was `GLIBCXX_3.4.30`.
- **A SECOND, independent floor: glibc `GLIBC_2.39`.** The noble build also
  raises the glibc requirement. Measured on the noble build: the module binary
  itself needs `GLIBC_2.38`, and the bundled `libviamsdk.so` needs `GLIBC_2.39`,
  so the **net glibc floor is `2.39`**. An older robot fails to load with
  `version 'GLIBC_2.39' not found`. This is a *separate axis* from libstdc++:
  `-static-libstdc++ -static-libgcc` does nothing for it, and — unlike
  libstdc++ — glibc **cannot be bundled** out of the floor (see below). **Net:
  the deployment target must be noble-class or newer on BOTH axes** — glibc
  ≥ `2.39` AND libstdc++ ≥ `GLIBCXX_3.4.32`. (Same yaskawa precedent: a noble
  build of `libviamsdk.so` sets the identical glibc floor, so a robot that runs
  a shipping noble Viam module already meets it.)
- **Why the module's own static linking doesn't remove it.** Even if the module
  binary links the C++ runtime statically (`-static-libstdc++ -static-libgcc`),
  that only covers *our* code. The module bundles and loads the Viam SDK's
  **shared** `libviamsdk.so` (via `$ORIGIN` RPATH out of `module.tar.gz`), and a
  shared library carries its own `DT_NEEDED` dependency on the system
  `libstdc++.so.6` — which a consumer binary's static-link flags cannot strip.
  So `libviamsdk.so`'s `GLIBCXX_3.4.32` requirement sets the floor regardless of
  how the module's own code is linked. (Measured on the noble build of
  `libviamsdk.so`; consistent with the GCC-13 toolchain.)
- **Precedent — this is not a new burden on the fleet.** The Viam
  `yaskawa-robots` C++ module is built on the **same** base image and SDK
  (`ubuntu:noble`, `viam-cpp-sdk` `releases/v0.31.0` — identical pins to ours:
  `reference-src/yaskawa-robots/Dockerfile`, `CMakeLists.txt:91`/`:104` vs our
  `Dockerfile`, `CMakeLists.txt:145`). Because the GLIBCXX requirement of a
  shared library is a deterministic function of (SDK source, compiler,
  libstdc++ headers) and all three are identical, yaskawa's deployed
  `libviamsdk.so` requires the **same `GLIBCXX_3.4.32` floor**. So any robot that
  can run a shipping noble-built Viam module already meets this floor — adopting
  it puts us no worse off than an existing deployed module.
- **For a sub-noble target fleet — bundling does NOT suffice; a sysroot rebuild
  is required (not applied).** Bundling `libstdc++.so.6` inside `module.tar.gz`
  (via `$ORIGIN` RPATH) clears the *libstdc++* floor only. It does **not** clear
  the glibc floor: glibc is not the constraint-direction people expect. glibc's
  compatibility guarantee is **forward** (a binary built against an *older* glibc
  runs on a *newer* one) — but we build on noble (`GLIBC_2.39`) and run on the
  *target*, so an *older* target is the unsupported direction. And glibc
  **cannot be bundled** the way libstdc++ can: `ld.so` and `libc.so.6` are
  selected by the kernel at `exec`, before `$ORIGIN`/RPATH interposition applies,
  so a bundled libc is not used for the program loader. A `module.tar.gz` that
  bundles only `libstdc++.so.6` would clear `GLIBCXX_3.4.32` and then still
  **fail at exec on `GLIBC_2.39 not found`**. The real way to deploy to a
  sub-noble fleet is to **rebuild the module against the target's (older) glibc**
  — i.e. build in a sysroot / older base image matching the fleet's glibc — not
  to bundle. This is **not applied by default** (the noble floor is
  precedented-safe via yaskawa); it is the documented path if a target fleet is
  known to be below the floor.

**Verify the module's required floors** (compute the max over the binary AND
every bundled `.so` — the static-linked binary alone *understates* the floor; in
practice `libviamsdk.so` dominates both axes):

```
# libstdc++ (GLIBCXX) floor:
objdump -T <module-binary> libviamsdk.so* | grep -o 'GLIBCXX_[0-9.]*' | sort -V | tail -1
# glibc (GLIBC) floor:
objdump -T <module-binary> libviamsdk.so* | grep -o 'GLIBC_[0-9.]*'   | sort -V | tail -1
```
Expected on the noble build: `GLIBCXX_3.4.32` and `GLIBC_2.39`. **Verify a
target robot meets both:** `strings /usr/lib/*/libstdc++.so.6 | grep -o
'GLIBCXX_[0-9.]*' | sort -V | tail -1` must be ≥ `GLIBCXX_3.4.32`, and `ldd
--version` (or the glibc on the robot) must be ≥ `2.39`. After deploy, `ldd
<module-binary>` must resolve cleanly (no `not found`).

---

## Verifying the grant

After applying any of the above, confirm the caps actually reached the **running
module process** (not just `viam-server`).

1. Find the module process PID. It is a child of `viam-server`; its command line
   contains the module entrypoint:
   ```bash
   pgrep -af <module-binary-name>
   ```

2. Inspect its effective/permitted/ambient capabilities:
   ```bash
   getpcaps <pid>
   # Expect: cap_net_raw,cap_net_admin,cap_sys_nice,cap_ipc_lock in eff/perm (and amb for the systemd path)
   ```

3. Or read the kernel status directly — the `Cap*` lines are hex bitmasks:
   ```bash
   grep -E 'Cap(Inh|Prm|Eff|Bnd|Amb)' /proc/<pid>/status
   ```
   Decode a mask with:
   ```bash
   capsh --decode=<hex-from-CapEff>
   # e.g. capsh --decode=000000000000300c
   ```
   You should see `cap_net_raw`, `cap_net_admin`, `cap_sys_nice`, `cap_ipc_lock`
   in `CapEff` (effective) and `CapPrm` (permitted). For the systemd ambient path
   they also appear in `CapAmb`.

4. Confirm the rlimits on the running process:
   ```bash
   grep -E 'Max realtime priority|Max locked memory' /proc/<pid>/limits
   # Max realtime priority   99  99
   # Max locked memory       unlimited  unlimited
   ```

5. End-to-end: if the caps are correct the module's `init` opens the NIC and sets
   `SCHED_FIFO` without throwing. A missing `CAP_NET_RAW` surfaces as the clear
   error: `failed to open EtherCAT interface 'ethX': need CAP_NET_RAW (run setcap
   or as root)`. A missing `CAP_SYS_NICE` surfaces as `real-time scheduling
   unavailable: …; set require_realtime=false to run best-effort`.
