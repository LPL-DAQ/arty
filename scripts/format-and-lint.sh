#!/bin/bash

set -euxo pipefail

# C/C++
find clover carl \( -iname '*.h' -o -iname '*.cpp' \) | xargs clang-format -i -style=file
# TODO: Add cppcheck/etc

# Python
ruff check --fix
ruff format

# TODO: Add shellcheck
