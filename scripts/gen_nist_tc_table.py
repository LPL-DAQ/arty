"""
Run as follows:

```
uv --project ~/arty/scripts run ~/arty/scripts/gen_nist_tc_table.py <k or t> <min_deg_c> <max_deg_c> <lut_size>
```

Existing clover LUTs were generated as follows:
```
uv --project ~/arty/scripts run ~/arty/scripts/gen_nist_tc_table.py k -200 400 200
uv --project ~/arty/scripts run ~/arty/scripts/gen_nist_tc_table.py t -200 50 100
```
"""

import csv
import sys

import numpy as np
import plotly.express as px
import polars as pl
import plotly.graph_objects as go

tc_type = sys.argv[1].lower()
min_deg_c = int(sys.argv[2])
max_deg_c = int(sys.argv[3])
lut_size = int(sys.argv[4])

# Get table path
if tc_type == 'k':
    table_path = 'scripts/lut_data/tc_k_type_table.txt'
elif tc_type == 't':
    table_path = 'scripts/lut_data/tc_t_type_table.txt'
else:
    raise ValueError('TC type must be k or t')

# Parse file
df = None
with open(table_path) as f:
    negative = True
    for line in f:
        vals = line.split()
        if len(vals) == 0:
            continue

        if vals[0] == '°C':
            if vals[2] == '1':
                negative = False
            elif vals[2] == '-1':
                negative = True
            else:
                raise ValueError(f'Bad line: {vals}')
            continue

        try:
            temp_tens = float(vals[0])
        except Exception as _:  # noqa: BLE001, S112
            continue

        voltages = [float(v) for v in vals[1:11]]
        if negative:
            temps = [temp_tens - i for i in range(len(voltages))]
        else:
            temps = [temp_tens + i for i in range(len(voltages))]

        print(f'temps: {temps}')
        print(f'voltages: {voltages}')

        partial_df = pl.DataFrame(
            {
                'temp': temps,
                'voltage': voltages,
            }
        )
        if df is None:
            df = partial_df
        else:
            df = df.extend(
                pl.DataFrame(
                    {
                        'temp': temps,
                        'voltage': voltages,
                    }
                )
            )

# Filter temp and convert mV to V
df = (
    df.sort('temp')
    .filter((pl.col('temp') >= min_deg_c) & (pl.col('temp') <= max_deg_c))
    .with_columns(voltage=pl.col('voltage') / 1000)
)
df_original = df

# Required for correct interpolation at ends
EPSILON = 0.000000001
min_voltage = df.min()[0, 'voltage'] + EPSILON
max_voltage = df.max()[0, 'voltage'] - EPSILON

# Interpolate table so voltage is at fixed increments
df = (
    df.with_columns(keep=pl.lit(value=False))
    .extend(
        pl.DataFrame(
            {
                'temp': pl.repeat(pl.lit(None), n=lut_size, eager=True),
                'voltage': pl.linear_space(
                    min_voltage, max_voltage, num_samples=lut_size, eager=True
                ),
                'keep': pl.repeat(pl.lit(value=True), n=lut_size, eager=True),
            }
        )
    )
    .sort('voltage')
    .with_columns(temp=pl.col('temp').interpolate_by('voltage'))
    .filter(pl.col('keep'))
    .with_columns(diff=pl.col('voltage').diff())
)

voltage_increment = df['diff'].drop_nulls().mean()

fig = px.line(df, x='temp', y='voltage')
fig = fig.add_trace(go.Scatter(x=df_original['temp'], y=df_original['voltage']))
fig.show(renderer='browser')

# Write lookup table
with open(f'scripts/lut_data/tc_{tc_type}_type_v_to_deg_c_lut.csv', '+w') as f:
    for temp in df['temp']:
        f.write(f'{temp}\n')

print('')
print(f'voltage: min={min_voltage}, max={max_voltage}, step={voltage_increment}, count={lut_size}')
print('')
print('RUN THE FOLLOWING TO GENERATE THE LOOKUP TABLE:')
print(
    f'uv --project ~/arty/scripts run ~/arty/scripts/gen_lookup_table_1d.py tc_{tc_type}_type_v_to_deg_c_lut {lut_size} {min_voltage} {max_voltage} {voltage_increment} scripts/lut_data/tc_{tc_type}_type_v_to_deg_c_lut.csv clover/src/lut/tc_{tc_type}_type_v_to_deg_c_lut.h'
)
print('')
