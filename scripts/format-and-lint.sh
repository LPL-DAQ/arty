#!/bin/bash

set -euxo pipefail

# C/C++
find clover carl \( -iname '*.h' -o -iname '*.cpp' \) | xargs clang-format -i -style=file
# TODO: Add cppcheck/etc

# Rust
cargo fmt

# Python
ruff check --fix
ruff format

# TODO: Add shellcheck
