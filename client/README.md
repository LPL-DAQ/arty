# Clover client

The ground-station CLI for the Clover flight computer: valve control and sequences (`vlvseq`),
sensor tables (`cfgana`), valve config (`cfgv`), live telemetry (`status`) and CSV recording.

Everything the client uses is in this folder. Other tools in `scripts/` are not part of it.

| Path | What it is |
| --- | --- |
| `client-new.py` | The client. Start here. |
| `sensor_configs/` | Sensor tables sent with `cfgana`. Edit biases here after zeroing. |
| `sequences/` | Throttle and other control sequences the client uploads. |
| `data/` | CSV recordings and plots (created when you record). |
| `app_paths.py`, `valve_status.py`, `valve_sequence_cli.py`, `named_sequences.py`, `sequence_countdown.py`, `clover_pb2.py` | Modules the client imports. |
| `tools/` | Developer tools: `fake_telemetry.py` (a fake board) and `test_clover_api.py`. |

The client finds these files relative to itself, so it doesn't matter which folder you start it from.

## Run it (Windows, macOS or Linux)

1. Install [uv](https://docs.astral.sh/uv/getting-started/installation/) once.
2. Get the repo: `git clone https://github.com/LPL-DAQ/arty.git`, or GitHub's *Code → Download ZIP*.
3. From the repo:

   ```sh
   cd client
   uv run client-new.py
   ```

uv installs a suitable Python and the five libraries the client needs on the first run; no compiler
is required. To update, `git pull` (or download the ZIP again).

Without uv: with Python 3.12+, `pip install "plotext<6" plotly prompt-toolkit protobuf rich`, then
`python client-new.py`.

Common options: `--host 169.254.99.99` (board address, the default) and `--no-data` (don't listen
for telemetry).

### Two windows at once

Only one client can receive telemetry. Start the window that runs `status` normally and the
`vlvseq` window with `--no-data`. The board accepts two connections in total, so close any other
client first.

## Keep it in sync with the firmware

- **Valve map:** the client reads `clover/src/sequencing/valve_map.json`, the same file the firmware
  is built with, so pull the same commit you flashed.
- **Protocol:** `clover_pb2.py` is generated from `api/clover.proto`. After changing the proto, run
  from the repo root: `protoc --python_out=client --proto_path=api api/clover.proto`.

## First run on a new laptop

- **Firewall:** allow Python to accept incoming connections when macOS or Windows asks (private
  networks on Windows), or telemetry won't arrive.
- **Network:** the laptop's Ethernet adapter must be on the board's link-local network (169.254.x.x).
