#!/usr/bin/bash

set -e

west init -l /home/lpl/clover
cd /home/lpl || exit 1
west update
uv pip install -r /home/lpl/zephyr/scripts/requirements.txt
west zephyr-export
west sdk install -t arm-zephyr-eabi

# Set up flasherd
cd /home/lpl/arty || exit 1
cargo build --package flasherd-client --release
mkdir -p /home/lpl/arty/bin
cp -f /home/lpl/arty/target/release/flasherd-client /home/lpl/arty/bin/flasherd-client
