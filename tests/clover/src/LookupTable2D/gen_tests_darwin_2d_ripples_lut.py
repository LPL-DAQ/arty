"""
Generates a lookup table used in tests/clover/src/LookupTable2D_test_ripples.cpp.
Approximates the function: f(x, y) = sin(sqrt((x+3)^2 + (y-1)^2)) * (x + 0.2 * y), x ∈ [-4, 5], y ∈ [-5, 3]

"""

import math

x = -4.0
y = -5.0
bps = []
for i in range(901):
    row = []
    for j in range(801):
        row.append(math.sin(math.sqrt((x + 3) ** 2 + (y - 1) ** 2)) * (x + 0.2 * y))
        y += 0.01
    bps.append(row)
    x += 0.01
    y = -5.0
    print(x)

with open('tests/clover/src/LookupTable2D/LookupTable2D_test_ripples.csv', '+w') as f:
    for row in bps:
        f.write(','.join(str(x) for x in row) + '\n')

print('x_low: -4, x_high: 5, y_low: -5, y_high: 3')
