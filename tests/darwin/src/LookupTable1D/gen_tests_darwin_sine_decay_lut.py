"""
Generates a lookup table used in tests/darwin/src/LookupTable1D_test_sine_decay.cpp.
Approximates the function: f(x) = sin(20 * x) / (x + 1.1), x ∈ [-1, 5]

"""

import math

x = -1.0
bps = []
for _ in range(6001):
    bps.append(math.sin(20 * x) / (x + 1.1))
    x += 0.001

with open('tests/darwin/src/LookupTable1D_test_sine_decay.csv', '+w') as f:
    f.write('\n'.join(str(x) for x in bps) + '\n')

print(f'num bps: {len(bps)}, x_low: -1, x_high: 5')
