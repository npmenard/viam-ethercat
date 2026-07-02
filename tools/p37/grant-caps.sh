#!/usr/bin/env bash
# #37 prep: grant the EtherCAT/RT capabilities the module needs (docs/deployment-capabilities.md).
# The module (exec'd by viam-server) needs: CAP_NET_RAW + CAP_NET_ADMIN (SOEM raw AF_PACKET socket
# on the NIC), CAP_SYS_NICE (SCHED_FIFO RT thread), CAP_IPC_LOCK (mlockall). require_realtime=true
# in the real-HW config -> without CAP_SYS_NICE the module THROWS at start().
#
# This box: setcap present (/sbin/setcap, libcap2-bin). viam-server NOT yet installed.
set -euo pipefail

MODE="${1:-help}"

case "$MODE" in
  systemd)   # RECOMMENDED: ambient caps on the viam-server (or viam-agent) unit -- survives module re-deploys.
    UNIT="${2:-viam-server}"   # pass 'viam-agent' if you run the agent
    echo "Run (as root), then edit the drop-in:"
    echo "  sudo systemctl edit ${UNIT}"
    cat <<CONF

  # --- paste into the [Service] section of the drop-in ---
  [Service]
  AmbientCapabilities=CAP_NET_RAW CAP_NET_ADMIN CAP_SYS_NICE CAP_IPC_LOCK
  CapabilityBoundingSet=CAP_NET_RAW CAP_NET_ADMIN CAP_SYS_NICE CAP_IPC_LOCK
  LimitRTPRIO=99
  LimitMEMLOCK=infinity
  # --- end ---
CONF
    echo "  sudo systemctl daemon-reload && sudo systemctl restart ${UNIT}"
    ;;
  setcap)    # FALLBACK: file caps on the unpacked module entrypoint binary.
    BIN="${2:?usage: grant-caps.sh setcap /abs/path/to/unpacked/module/entrypoint}"
    echo "sudo setcap 'cap_net_raw,cap_net_admin,cap_sys_nice,cap_ipc_lock+ep' '${BIN}'"
    echo "verify: getcap '${BIN}'"
    echo "NOTE: setcap is lost on every module re-deploy (new unpacked binary) -> re-run after each build."
    ;;
  verify)
    BIN="${2:?usage: grant-caps.sh verify /abs/path/to/entrypoint}"; getcap "${BIN}" || true
    echo "rlimits (this shell): RTPRIO=$(ulimit -r 2>/dev/null||echo ?) MEMLOCK=$(ulimit -l 2>/dev/null||echo ?)KB"
    ;;
  *)
    echo "usage: $0 {systemd [viam-server|viam-agent] | setcap <entrypoint> | verify <entrypoint>}"
    echo "recommended: systemd ambient (survives re-deploys). See docs/deployment-capabilities.md."
    ;;
esac
