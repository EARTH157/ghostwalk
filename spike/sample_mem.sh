#!/bin/bash
# Every ~1 s: epoch, system RAM used MiB, swap used MiB, GPU mem used MiB, spike RSS MiB, swap-in/out pages/s (vmstat).
while true; do
  rss=$(ps -o rss= -p "$(pgrep -d, -f m0_spike.py)" 2>/dev/null | awk '{s+=$1} END{print int(s/1024)}')
  sio=$(vmstat 1 2 | tail -1 | awk '{print $7, $8}')
  echo "$(date +%s) $(free -m | awk '/^Mem/{u=$3} /^Swap/{s=$3} END{print u, s}') $(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits) ${rss:-0} $sio"
done
