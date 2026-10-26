#!/bin/bash
# Log the Teensy TC probe with timestamps. Usage: ./log.sh [minutes]   (default 30). Ctrl-C to stop.
# Close `pio device monitor` first: only one program can read the port.
port=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
[ -z "$port" ] && { echo "no /dev/cu.usbmodem*: Teensy not connected or sketch not running"; exit 1; }
out="logs/tc_$(date +%Y%m%d_%H%M%S).txt"; mkdir -p logs
echo "logging $port -> $out"
perl -MTime::HiRes=time -e 'alarm($ARGV[1]*60); open(my $f,"<",$ARGV[0]) or die "$!\n"; $|=1;
  while(<$f>){ s/\r//; printf "%.2f %s", time, $_ } print STDERR "port closed (monitor opened? unplugged?)\n"' "$port" "${1:-30}" | tee "$out"
