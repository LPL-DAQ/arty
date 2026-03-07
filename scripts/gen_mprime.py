import polars as pl

df = pl.read_csv('MPrime_March5Round2.csv', schema_overrides=[pl.datatypes.Float64, pl.datatypes.Float64, pl.datatypes.Float64, pl.datatypes.Float64])

thrust_axis = df.unique(subset=pl.col('Target_Thrust_lbf')).sort(by='Target_Thrust_lbf')['Target_Thrust_lbf']
of_axis = df.unique(subset=pl.col('Target_OF')).sort(by='Target_OF')['Target_OF']

print('Thrust axis:')
text = ''
for val in thrust_axis:
    text += f'{val}f, '
print(text)

print('MR axis:')
text = ''
for val in of_axis:
    text += f'{val}f, '
print(text)

print(f'fuel')
text = ''
for val in thrust_axis:
    text += '{'
    for val2 in of_axis:
        row = df.filter([pl.col("Target_Thrust_lbf") == val, pl.col("Target_OF") == val2])
        text += f'{(row[0, "Req_Fuel_Valve_deg"])}f, '
    text += '}, '
print(text)

print(f'lox')
text = ''
for val in thrust_axis:
    text += '{'
    for val2 in of_axis:
        row = df.filter([pl.col("Target_Thrust_lbf") == val, pl.col("Target_OF") == val2])
        text += f'{(row[0, "Req_LOx_Valve_deg"])}f, '
    text += '}, '
print(text)
