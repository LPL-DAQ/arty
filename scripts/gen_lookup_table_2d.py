"""
Generates a 2D lookup table. Input must be a headerless CSV where increasing rows correspond to increasing along
the X axis and increasing columns correspond to increasing along the Y axis.

Call this script as such from ~/arty --

```
uv --project ~/arty/scripts run ~/arty/scripts/gen_lookup_table_2d.py <snake_case_name> <x_len> <x_min> <x_max> <x_gap> <y_len> <y_min> <y_max> <y_gap> <input_file_path> <output_file_path>
```
"""

import csv
import math
import string
import sys

EPSILON = 0.0001
LOWER_SNAKE_CASE = string.ascii_lowercase + string.digits + '_'

if len(sys.argv) != 12:
    print(
        'usage: uv --project ~/arty/scripts run ~/arty/scripts/gen_lookup_table_2d.py <snake_case_name> <x_len> <x_min> <x_max> <x_gap> <y_len> <y_min> <y_max> <y_gap> <input_file_path> <output_file_path>'
    )
    sys.exit(1)

name = sys.argv[1]
x_len = int(sys.argv[2])
x_min = float(sys.argv[3])
x_max = float(sys.argv[4])
x_gap = float(sys.argv[5])
y_len = int(sys.argv[6])
y_min = float(sys.argv[7])
y_max = float(sys.argv[8])
y_gap = float(sys.argv[9])
input_file_path = sys.argv[10]
output_file_path = sys.argv[11]

# Validate input
if len(name) == 0:
    raise ValueError(f'name `{name}` must be non-empty')
for c in name:
    if c not in LOWER_SNAKE_CASE:
        raise ValueError(f'name `{name}` must be in lower snake case')
if x_min >= x_max:
    raise ValueError(f'x_min `{x_min}` must be less than x_max `{x_max}`')
if x_gap <= 0:
    raise ValueError(f'x_gap `{x_gap}` must be positive')
if math.fabs(x_min + (x_len - 1) * x_gap - x_max) > EPSILON:
    raise ValueError(
        f'x_max `{x_max}` must match expected given x_len `{x_len}`, x_gap `{x_gap}`, and x_min `{x_min}`'
    )
if y_min >= y_max:
    raise ValueError(f'y_min `{y_min}` must be less than y_max `{y_max}`')
if y_gap <= 0:
    raise ValueError(f'y_gap `{y_gap}` must be positive')
if math.fabs(y_min + (y_len - 1) * y_gap - y_max) > EPSILON:
    raise ValueError(
        f'y_max `{y_max}` must match expected given y_len `{y_len}`, y_gap `{y_gap}`, and y_min `{y_min}`'
    )

# Parse input file
breakpoints = []
with open(input_file_path) as f:
    reader = csv.reader(f)
    for row in reader:
        if len(row) != y_len:
            raise ValueError(
                f'Bad row in input file at `{input_file_path}`: {row} should match y_len `{y_len}`'
            )
        breakpoints.append([float(x) for x in row])

if len(breakpoints) != x_len:
    raise ValueError(
        f'Number of rows in input file at `{input_file_path}` must match given x_len `{x_len}`'
    )

# Build output file
name_upper_snake = name.upper()
name_upper_camel = ''.join(word[0].upper() + word[1:] for word in name.split('_'))

bps_text = (
    '{'
    + ', '.join(
        [
            f'std::array<float, {name_upper_snake}_Y_LEN>{{'
            + ', '.join([f'{bp:.10f}f' for bp in row])
            + '}'
            for row in breakpoints
        ]
    )
    + '}'
)

payload = f"""/*
>>> GENERATED FILE <<<

Re-create this whenever {input_file_path} changes by running from the arty directory:

```
uv --project ~/arty/scripts run ~/arty/scripts/gen_lookup_table_2d.py {name} {x_len} {x_min} {x_max} {x_gap} {y_len} {y_min} {y_max} {y_gap} {input_file_path} {output_file_path}
```
*/

#pragma once

#include <array>
#include "LookupTable2D.h"

constexpr int {name_upper_snake}_X_LEN = {x_len};
constexpr float {name_upper_snake}_X_MIN = {x_min:.10f}f;
constexpr float {name_upper_snake}_X_MAX = {x_max:.10f}f;
constexpr float {name_upper_snake}_X_GAP = {x_gap:.10f}f;

constexpr int {name_upper_snake}_Y_LEN = {y_len};
constexpr float {name_upper_snake}_Y_MIN = {y_min:.10f}f;
constexpr float {name_upper_snake}_Y_MAX = {y_max:.10f}f;
constexpr float {name_upper_snake}_Y_GAP = {y_gap:.10f}f;

constexpr std::array<std::array<float, {name_upper_snake}_Y_LEN>, {name_upper_snake}_X_LEN> {name_upper_snake}_BPS {bps_text};

typedef LookupTable2D<{name_upper_snake}_X_LEN, {name_upper_snake}_X_MIN, {name_upper_snake}_X_MAX, {name_upper_snake}_X_GAP, {name_upper_snake}_Y_LEN, {name_upper_snake}_Y_MIN, {name_upper_snake}_Y_MAX, {name_upper_snake}_Y_GAP, {name_upper_snake}_BPS> {name_upper_camel};
"""
with open(output_file_path, '+w') as f:
    f.write(payload)

print(f'>>>> wrote payload to {output_file_path} <<<<')
print(payload)
