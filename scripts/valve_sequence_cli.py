"""Interactive named-valve-sequence mode, using the existing board connection."""
from datetime import datetime
import threading

from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

import clover_pb2
import named_sequences


class ValveSequenceMode:
    def __init__(self, connection, console, poll_interval=0.2):
        self.connection = connection
        self.console = console
        self.poll_interval = poll_interval
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.broken = False
        self.last_status = None

    def show_status(self, info, force=False):
        """Called under lock, keeping command replies and progress in order."""
        signature = (info.state, info.name, info.step, info.total_steps, info.comment, info.error)
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

    def execute(self, command, quiet=False):
        try:
            request = named_sequences.make_request('sequence ' + command)
        except ValueError:
            self.console.print('Use: list | run <name> | status | abort | back', style='yellow')
            return False
        with self.lock:
            if self.broken:
                if not quiet:
                    self.console.print('Connection unavailable. Use back and restart the client.', style='red')
                return False
            previous_timeout = self.connection.gettimeout()
            try:
                self.connection.settimeout(3)
                info = named_sequences.exchange(self.connection, request)
            except named_sequences.RequestRejected as exc:
                self.console.print(Text(f'Rejected: {exc}', style='red'))
                return False
            except Exception as exc:
                self.broken = True
                self.connection.close()
                self.console.print(Text(f'Connection failed: {exc}', style='red'))
                self.console.print('No command was retried. Restart the client and check status before another run.',
                                   style='yellow')
                return False
            finally:
                if not self.broken:
                    self.connection.settimeout(previous_timeout)
            operation = None if request.HasField('abort') else request.named_sequence.operation
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
            self.show_status(info, force=operation == clover_pb2.NamedSequenceRequest.STATUS and not quiet)
            return True

    def watch(self):
        while not self.done.wait(self.poll_interval):
            if not self.execute('status', quiet=True):
                break

    def run(self, session=None):
        session = session or PromptSession()
        self.console.print(Panel(
            Text('list           Show stored sequences\n'
                 'run <name>     Start a sequence\n'
                 'status         Show current state, step and comment\n'
                 'abort          Abort sequence and request outputs off\n'
                 'back / exit    Return to CMD (does not stop the board)\n\n'
                 'Progress updates appear automatically while this mode is open.'),
            title='Valve sequences — vlvseq', border_style='cyan'))
        # Redraw the prompt after background output instead of mixing logs with input.
        with patch_stdout(raw=True):
            if not self.execute('status'):
                return
            watcher = threading.Thread(target=self.watch, daemon=True, name='valve-sequence-status')
            watcher.start()
            try:
                while True:
                    try:
                        command = session.prompt('\n  VLVSEQ> ').strip()
                    except KeyboardInterrupt:
                        self.console.print('Use abort to abort the sequence, or back to leave this mode.', style='yellow')
                        continue
                    except EOFError:
                        break
                    if command.lower() in ('back', 'exit', 'quit', 'q'):
                        break
                    if command.lower() in ('help', '?', 'h'):
                        self.console.print('list | run <name> | status | abort | back')
                    elif command:
                        self.execute(command)
            finally:
                self.done.set()
                watcher.join()
        self.console.print('Returned to CMD. Leaving vlvseq does not cancel a running sequence.', style='dim')
