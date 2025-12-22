#!/usr/bin/bash

set -e

# Set up flasherd
cd /home/lpl/arty || exit 1
cargo build --package flasherd-client --release
mkdir -p /home/lpl/arty/bin
cp -f /home/lpl/arty/target/release/flasherd-client /home/lpl/arty/bin/flasherd-client
