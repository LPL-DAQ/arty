"""Interactive named-valve-sequence mode, using the existing board connection."""
from datetime import datetime
import threading
import json
import math
import sys
import time

from prompt_toolkit import PromptSession
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.patch_stdout import patch_stdout
from rich import box
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

import clover_pb2
import named_sequences
from valve_status import MAP_PATH


class ValveSequenceMode:
    def __init__(self, connection, console, poll_interval=0.2, client_commands=None):
        self.client_commands = client_commands if client_commands is not None else {}
        self.connection = connection
        self.console = console
        self.poll_interval = poll_interval
        # Polling and prompt commands share one socket; lock the whole transaction.
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.broken = False
        self.last_status = None
        self.pending_run = None
        self.countdown_last = None
        self.override_supported = None
        self.latest_info = None

    def show_status(self, info, force=False):
        """Called under lock, keeping command replies and progress in order."""
        self.latest_info = info
        self.override_supported = info.HasField('manual_override_channels')
        # Poll frequently but print only transitions unless status was explicitly requested.
        signature = (info.state, info.name, info.step, info.total_steps, info.comment, info.error,
                     info.powered_channels, info.known_channels,
                     info.manual_override_channels, info.manual_powered_channels,
                     info.sequence_requested_channels)
        if signature == self.last_status and not force:
            return
        previous = self.last_status
        self.last_status = signature
        styles = {'running': 'cyan', 'aborting': 'yellow', 'aborted': 'yellow',
                  'complete': 'green', 'ready': 'green', 'faulted': 'bold red'}
        line = Text(f'[{datetime.now():%H:%M:%S}] ', style='dim')
        line.append(info.state.upper(), style=styles.get(info.state, 'white'))
        if info.name:
            line.append(f'  {info.name}  |  step {info.step}/{info.total_steps}')
        if info.error:
            line.append(f'  |  GPIO error {info.error}; reset required', style='bold red')
        self.console.print(line)
        if info.comment and (force or previous is None or signature[1:5] != previous[1:5]):
            # Text prevents comments containing [] or terminal escapes becoming formatting.
            self.console.print(Text('    ' + info.comment))

    def cancel_pending(self):
        with self.lock:
            if self.pending_run is not None:
                self.pending_run = None
                self.console.print('Cancelled. Sequence was not started; connection remains open.')

    def schedule_run(self, name, seconds=10):
        with self.lock:
            if self.pending_run is not None:
                self.console.print('A start is already pending. Press Space or Q to cancel first.', style='yellow')
                return
            self.pending_run = (name, time.monotonic() + seconds)
            self.countdown_last = None
            self.console.print(Text(f'Starting {name} in {seconds}s. Manual control is blocked; '
                                    'press Space or Q at an empty prompt to cancel.'))

    def advance_countdown(self):
        with self.lock:
            if self.pending_run is None:
                return
            name, deadline = self.pending_run
            remaining = max(0, math.ceil(deadline - time.monotonic()))
            if remaining == 0:
                self.pending_run = None
                self.execute('run ' + name)
            elif remaining != self.countdown_last:
                self.countdown_last = remaining
                self.console.print(Text(f'Starting {name} in {remaining}...'))

    def manual_blocked(self):
        with self.lock:
            if self.pending_run is not None:
                reason = 'A sequence countdown is active. Cancel it or wait for the run to finish.'
            elif self.latest_info is not None and self.latest_info.state in ('running', 'aborting'):
                reason = 'A sequence is running. Wait for completion or abort it.'
            else:
                return False
            self.console.print(Panel(Text(reason, style='bold white'), title='MANUAL CONTROL BLOCKED',
                                     border_style='bold red'))
            return True

    def manual_prompt(self, session):
        if self.manual_blocked():
            return
        try:
            mapping = json.loads(MAP_PATH.read_text())
            valves = [(name, channel) for name, channel in mapping.items() if channel is not None]
            for index, (name, channel) in enumerate(valves, 1):
                self.console.print(Text(f'[{index}] {name} (channel {channel})'))
            choice = session.prompt('Select valve (q to cancel): ').strip()
            if choice.lower() in ('q', 'back', 'exit'):
                return
            index = int(choice) - 1
            if not 0 <= index < len(valves):
                raise ValueError('Invalid valve selection')
            name, channel = valves[index]
            self.console.print(Panel(Text(f'{name}\nGPIO channel {channel}', justify='center', style='bold bright_cyan'),
                                     title='SELECTED VALVE', box=box.DOUBLE,
                                     border_style='bright_cyan', padding=(1, 4), width=60))
            action = session.prompt('1: ON | 2: OFF | q: cancel > ').strip().lower()
            if action in ('q', 'back', 'exit'):
                return
            actions = {'1': 'on', '2': 'off'}
            if action not in actions:
                raise ValueError('Select 1 or 2')
            self.execute(actions[action] + ' ' + name)
        except (KeyboardInterrupt, EOFError):
            self.cancel_pending()
            self.console.print('Manual selection cancelled.')
        except (OSError, ValueError) as exc:
            self.console.print(Text(str(exc), style='yellow'))

    def execute(self, command, quiet=False):
        try:
            words = command.split() if isinstance(command, str) else []
            if isinstance(command, clover_pb2.Request):
                request = command
                if request.HasField('manual_valve') and request.manual_valve.release:
                    raise ValueError('Release is not available in vlvseq')
            elif len(words) == 2 and words[0].lower() in ('on', 'off'):
                mapping = json.loads(MAP_PATH.read_text())
                channel = mapping.get(words[1].upper())
                if type(channel) is not int or not 0 <= channel < 32:
                    raise ValueError('Unknown or unassigned valve')
                request = clover_pb2.Request()
                request.manual_valve.channel = channel
                request.manual_valve.powered = words[0].lower() == 'on'
            else:
                request = named_sequences.make_request('sequence ' + command)
        except (ValueError, OSError) as exc:
            self.console.print(Text(f'{exc}. Use list | run <name> | v | status | abort | back', style='yellow'))
            return False
        with self.lock:
            if self.broken:
                if not quiet:
                    self.console.print('Connection unavailable. Use back and restart the client.', style='red')
                return False
            if request.HasField('manual_valve'):
                if self.manual_blocked():
                    return False
                if self.override_supported is None and not self.execute('status', quiet=True):
                    return False
                if not self.override_supported:
                    self.console.print('Firmware lacks manual override support; rebuild and flash before manual control.', style='yellow')
                    return False
            previous_timeout = self.connection.gettimeout()
            try:
                self.connection.settimeout(3)
                info = named_sequences.exchange(self.connection, request)
            except named_sequences.RequestRejected as exc:
                if 'MANUAL CONTROL BLOCKED' in str(exc):
                    self.console.print(Panel(Text(str(exc), style='bold white'), title='MANUAL CONTROL BLOCKED', border_style='bold red'))
                else:
                    self.console.print(Text(f'Rejected: {exc}', style='red'))
                return False
            except Exception as exc:
                # After a framing/transport failure, a later reply could belong to this request.
                # Discard the connection rather than replaying a potentially accepted run.
                self.pending_run = None
                self.broken = True
                self.connection.close()
                self.console.print(Text(f'Connection failed: {exc}', style='red'))
                self.console.print('No command was retried. Restart the client and check status before another run.',
                                   style='yellow')
                return False
            finally:
                if not self.broken:
                    self.connection.settimeout(previous_timeout)
            manual = request.HasField('manual_valve')
            if manual:
                command = request.manual_valve
                self.client_commands[command.channel] = command.powered
                self.console.print(Text(f'Manual command accepted: channel {command.channel} '
                                        + ('ON' if command.powered else 'OFF')))
            operation = None if request.HasField('abort') or manual else request.named_sequence.operation
            if operation == clover_pb2.NamedSequenceRequest.LIST:
                table = Table(title='Stored valve sequences', show_header=True, min_width=30)
                table.add_column('Name', style='cyan')
                for name in info.names:
                    table.add_row(Text(name))
                self.console.print(table)
            elif operation == clover_pb2.NamedSequenceRequest.RUN:
                self.console.print(Text(f'Run accepted: {request.named_sequence.name}', style='green'))
            elif request.HasField('abort'):
                self.console.print('Abort requested; waiting for aborted or faulted status.', style='yellow')
            explicit_status = operation == clover_pb2.NamedSequenceRequest.STATUS and not quiet
            self.show_status(info, force=manual or explicit_status)
            return True

    def watch(self):
        """Observe board progress without making the UI responsible for sequence timing."""
        while not self.done.wait(self.poll_interval):
            self.advance_countdown()
            if not self.execute('status', quiet=True):
                break

    def run(self, session=None):
        bindings = KeyBindings()

        @bindings.add(' ')
        @bindings.add('q')
        @bindings.add('Q')
        def cancel_key(event):
            if self.pending_run is not None and not event.app.current_buffer.text:
                self.cancel_pending()
            else:
                event.app.current_buffer.insert_text(event.data)

        session = session or PromptSession(key_bindings=bindings)
        self.console.print(Panel(
            Text('list           Show stored sequences\n'
                 'run <name>     Start after a 10s countdown; manual locks immediately\n'
                 'v              Select a valve and ON/OFF\n'
                 'status         Show sequence state and progress\n'
                 'abort          Abort sequence and request outputs off\n'
                 'back / exit    Return to CMD (does not stop the board)\n\n'
                 'Progress updates appear automatically while this mode is open.'),
            title='Valve control — vlvseq', border_style='cyan'))
        # Redraw the prompt after background output instead of mixing logs with input.
        with patch_stdout(raw=True):
            if not self.execute('status', quiet=True):
                return
            watcher = threading.Thread(target=self.watch, daemon=True, name='valve-sequence-status')
            watcher.start()
            try:
                while True:
                    try:
                        command = session.prompt('\n  VLVSEQ> ').strip()
                    except KeyboardInterrupt:
                        self.cancel_pending()
                        self.console.print('Use abort to abort the sequence, or back to leave this mode.', style='yellow')
                        continue
                    except EOFError:
                        break
                    if command.lower() in ('back', 'exit', 'quit', 'q'):
                        break
                    if command.lower() in ('help', '?', 'h'):
                        self.console.print('list | run <name> | v | status | abort | back')
                    elif command:
                        words = command.split()
                        if command.lower() == 'v':
                            self.manual_prompt(session)
                        elif words[0].lower() not in ('list', 'run', 'status', 'abort'):
                            self.console.print('Use list | run <name> | v | status | abort | back', style='yellow')
                        elif len(words) == 2 and words[0].lower() == 'run' and sys.stdin.isatty():
                            self.schedule_run(words[1])
                        else:
                            if command.lower() == 'abort':
                                self.cancel_pending()
                            self.execute(command)
            finally:
                # Join before returning so the main CLI regains exclusive socket ownership.
                self.cancel_pending()
                self.done.set()
                watcher.join()
        self.console.print('Returned to CMD. Leaving vlvseq does not cancel a running sequence.', style='dim')
