#!/usr/bin/env bash
# viam-server lifecycle for the RDK validation campaign.
# The server runs as root (sudo) so the spawned module inherits every capability it
# needs (CAP_NET_RAW / NET_ADMIN / SYS_NICE / IPC_LOCK + RT scheduling).
# NOTE: loading the module against the real A6 AUTO-ENERGIZES the shaft.
set -euo pipefail

BIN=/home/viam/rdk/bin/Linux-x86_64/viam-server-static
CFG=/home/viam/rdk/viam.json
LOGDIR=/home/viam/rdk/logs
NIC=enp86s0

case "${1:-}" in
  start)
    mkdir -p "$LOGDIR"
    sudo ip link set "$NIC" up || true
    # carrier takes a few seconds after link-up
    for _ in $(seq 1 10); do
      ip link show "$NIC" | grep -q "state UP" && break
      sleep 1
    done
    LOG="$LOGDIR/viam-server-$(date +%Y%m%d-%H%M%S).log"
    ln -sf "$LOG" "$LOGDIR/latest.log"
    sudo nohup "$BIN" -config "$CFG" >"$LOG" 2>&1 &
    echo "viam-server starting; log: $LOG"
    ;;
  stop)
    sudo pkill -f viam-server-static || echo "not running"
    # the module tears down via Quick-Stop ramp -> de-energize; give it a moment
    sleep 3
    ;;
  status)
    pgrep -af viam-server-static || echo "not running"
    pgrep -af ethercat-servo || echo "module not running"
    ;;
  log)
    tail -n "${2:-50}" "$LOGDIR/latest.log"
    ;;
  *)
    echo "usage: $0 start|stop|status|log [n]"
    exit 2
    ;;
esac
