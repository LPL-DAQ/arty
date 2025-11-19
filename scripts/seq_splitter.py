# Run as less ~/ | uv run scripts/seq-splitter.py

import sys
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import datetime
import re
import os

print('Importing pandas, this can take a while the first time...')
import pandas as pd

LOGS_DIR = '/home/lpl/clover/scripts/data/sequences'
SEQ_FORMAT_VERSION = 'alpha'

print(f'Logging to: {LOGS_DIR}')
os.makedirs(LOGS_DIR, exist_ok=True)

print(f'Starting sequence splitter, format version: {SEQ_FORMAT_VERSION}')
print(f'Ready for sequences')
in_seq = False
log_text = ''
for line in sys.stdin:
    line = line.strip()
    if line == '>>>>SEQ START<<<<':
        in_seq = True
        seq_start = datetime.datetime.now()
        print(f'Starting sequence at: {seq_start}')

    elif line == '>>>>SEQ END<<<<':
        in_seq = False
        start_stamp = seq_start.isoformat()
        start_stamp = re.sub('[^0-9]', '_', start_stamp)
        log_path = f'{LOGS_DIR}/seq_{start_stamp}_ver_{SEQ_FORMAT_VERSION}.csv'
        with open(log_path, 'w') as log_file:
            log_file.write(log_text)
        log_text = ''

        print(f'Wrote sequence data to: {log_path}')

        print(f'Plotting data...')
        df = pd.read_csv(log_path)

        SUBPLOT_SPECS = [
            {
                'row': 1,
                'col': 1,
                'sensors': ['pt202', 'pt203', 'ptf401']
            },
            {
                'row': 2,
                'col': 1,
                'sensors': ['motor_pos', 'encoder_deg', 'motor_target']
                # 'sensors': ['motor_pos', 'motor_target', 'motor_velocity', 'motor_acceleration']
            },
        ]
        fig = make_subplots(2, 1, shared_xaxes=True, shared_yaxes=False, horizontal_spacing=0.05, vertical_spacing=0.05,
                            subplot_titles=['PTs', 'Motor'])
        for spec in SUBPLOT_SPECS:
            for sensor in spec['sensors']:
                fig.add_trace(go.Scatter(x=df['time'], y=df[sensor], name=sensor), row=spec['row'],
                              col=spec['col'])
        fig.update_layout({
            # 'template': 'plotly_light',
            'hoversubplots': 'axis',
            'hovermode': 'x',
            'grid': {
                'rows': 2,
                'columns': 1
            },
            'title': {
                'text': f'Sequence: {log_path}'
            }
        })
        fig.show(renderer='browser')

        print(f'Ready for next sequence')

    elif in_seq:
        log_text += line + '\n'
