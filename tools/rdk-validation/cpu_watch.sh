#!/usr/bin/env bash
# Sample the ethercat-servo module's CPU usage (user requirement: the module must
# not consume excessive CPU -- the RT loop sleeps between ticks via clock_nanosleep).
# Usage: ./cpu_watch.sh [seconds] (default 30); prints per-second %CPU + a summary.
set -euo pipefail

DUR="${1:-30}"
PID=$(pgrep -f 'bin/ethercat-servo' | head -1)
[ -n "$PID" ] || { echo "module not running"; exit 1; }
echo "watching pid $PID for ${DUR}s"
if command -v pidstat >/dev/null; then
  pidstat -p "$PID" 1 "$DUR" | tail -n 3
else
  # fallback: /proc sampling
  read -r utime0 stime0 < <(awk '{print $14, $15}' "/proc/$PID/stat")
  t0=$(date +%s.%N)
  sleep "$DUR"
  read -r utime1 stime1 < <(awk '{print $14, $15}' "/proc/$PID/stat")
  t1=$(date +%s.%N)
  hz=$(getconf CLK_TCK)
  awk -v u0="$utime0" -v s0="$stime0" -v u1="$utime1" -v s1="$stime1" \
      -v t0="$t0" -v t1="$t1" -v hz="$hz" \
      'BEGIN { printf "avg CPU: %.1f%%\n", 100*((u1-u0)+(s1-s0))/hz/(t1-t0) }'
fi
