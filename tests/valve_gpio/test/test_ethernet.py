"""Real TCP + nanopb interoperability tests using the C firmware with host GPIO adapters."""
import contextlib
import io
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from rich.console import Console

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0, str(REPO / 'scripts'))
import clover_pb2
import named_sequences
from valve_sequence_cli import ValveSequenceMode
from google.protobuf.internal.encoder import _VarintBytes


class EthernetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Use the nanopb module from the same west workspace as the firmware build.
        result = subprocess.run(['west', 'list', 'nanopb', '-f', '{abspath}'],
                                cwd=REPO, check=True, capture_output=True, text=True)
        cls.nanopb = Path(result.stdout.strip())
        cls.temp = tempfile.TemporaryDirectory()
        directory = cls.directory = Path(cls.temp.name)
        sequences = directory / 'sequences'
        sequences.mkdir()
        (sequences / 'alpha.json').write_text(json.dumps([
            {'duration_ms': 1000, 'channels': [0, 1], 'comment': 'Alpha "test" café %'},
            {'duration_ms': 10, 'channels': []}]))
        (sequences / 'MixedCase.json').write_text('[{"duration_ms":10,"channels":[2]}]')
        subprocess.run([sys.executable, str(ROOT / 'generate_sequence.py'), str(sequences),
                        str(directory / 'valve_sequence.h')], check=True)
        subprocess.run([sys.executable, str(cls.nanopb / 'generator/nanopb_generator.py'),
                        '-I', str(REPO / 'api'), '-D', str(directory),
                        str(REPO / 'api/clover.proto')], check=True, capture_output=True)
        for name in ['kernel.h', 'drivers/gpio.h', 'sys/util.h', 'logging/log.h', 'net/socket.h']:
            path = directory / 'zephyr' / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#include "host_zephyr.h"\n')
        engine = (ROOT / 'src/embedded_sequence.c').read_text().replace('#include "test_trace.h"', '')
        engine = engine.replace('int main(void)', 'int firmware_main(void)')
        server = (ROOT / 'src/sequence_server.c').read_text().replace('htons(19690)', 'htons(host_port)')
        harness = directory / 'host.c'
        harness.write_text('#include "host_zephyr.h"\n' + engine + '\n' + server + r'''
static void *engine_thread(void *unused) { (void)unused; firmware_main(); return NULL; }
int main(int argc, char **argv) {
    (void)argc; host_port = (unsigned)atoi(argv[1]);
    inject_fault = getenv("VALVE_INJECT_FAULT") != NULL;
    pthread_t thread; pthread_create(&thread, NULL, engine_thread, NULL);
    serve(NULL, NULL, NULL); return 0;
}
''')
        cls.binary = directory / 'server'
        subprocess.run(['cc', '-std=c11', '-pthread', '-I', str(directory),
                        '-I', str(ROOT / 'src'), '-I', str(ROOT / 'test'), '-I', str(cls.nanopb),
                        str(harness), str(ROOT / 'src/sequence_protocol.c'), str(directory / 'clover.pb.c'),
                        *[str(cls.nanopb / file) for file in ['pb_common.c', 'pb_decode.c', 'pb_encode.c']],
                        '-o', str(cls.binary)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    @contextlib.contextmanager
    def server(self, fault=False):
        with socket.socket() as reserve:
            reserve.bind(('127.0.0.1', 0))
            port = reserve.getsockname()[1]
        env = os.environ.copy()
        if fault:
            env['VALVE_INJECT_FAULT'] = '1'
        with tempfile.TemporaryFile(mode='w+') as logs:
            process = subprocess.Popen([str(self.binary), str(port)], stdout=logs, stderr=logs, env=env)
            try:
                deadline = time.monotonic() + 5
                while True:
                    try:
                        connection = socket.create_connection(('127.0.0.1', port), timeout=1)
                        break
                    except OSError:
                        if time.monotonic() > deadline or process.poll() is not None:
                            logs.seek(0)
                            self.fail('Host server did not start: ' + logs.read())
                        time.sleep(.01)
                with connection:
                    yield connection, port, logs
            finally:
                process.terminate()
                process.wait(timeout=5)

    def request(self, connection, command):
        return named_sequences.exchange(connection, named_sequences.make_request(command))

    def wait_state(self, connection, expected):
        deadline = time.monotonic() + 3
        while True:
            info = self.request(connection, 'sequence status')
            if info.state == expected:
                return info
            if time.monotonic() > deadline:
                self.fail(f'Expected {expected}, got {info}')
            time.sleep(.01)

    def test_named_runs_busy_stop_disconnect_and_client_cli(self):
        with self.server() as (connection, port, logs):
            info = self.request(connection, 'sequence list')
            self.assertEqual(list(info.names), ['MixedCase', 'alpha'])
            self.assertEqual(info.state, 'ready')
            self.assertEqual(info.step, 0)
            with self.assertRaisesRegex(ValueError, 'rejected'):
                self.request(connection, 'sequence run missing')
            self.request(connection, 'sequence run alpha')
            info = self.wait_state(connection, 'running')
            # Progress is populated asynchronously after RUN is queued.
            deadline = time.monotonic() + 3
            while not info.step:
                self.assertLess(time.monotonic(), deadline, 'Step did not start')
                info = self.request(connection, 'sequence status')
            self.assertEqual(info.comment, 'Alpha "test" café %')
            with self.assertRaisesRegex(ValueError, 'rejected'):
                self.request(connection, 'sequence run MixedCase')
            with socket.create_connection(('127.0.0.1', port), timeout=1) as second:
                self.request(second, 'sequence stop')
            self.wait_state(connection, 'stopped')
            self.request(connection, 'sequence run MixedCase')
            self.wait_state(connection, 'complete')
            # The production client routes its actual command via the same TCP server.
            cli = subprocess.run([sys.executable, str(REPO / 'scripts/client-new.py'),
                                  '--host', '127.0.0.1', '--port', str(port),
                                  '--command', 'sequence run MixedCase'], capture_output=True, text=True, timeout=10)
            self.assertEqual(cli.returncode, 0, cli.stdout + cli.stderr)
            self.wait_state(connection, 'complete')
            with socket.create_connection(('127.0.0.1', port), timeout=1) as second:
                self.request(second, 'sequence run alpha')
            # Closing the initiating connection leaves the board's timed run intact.
            self.wait_state(connection, 'complete')
            logs.seek(0)
            text = logs.read()
            self.assertLess(text.index('READY:'), text.index('ON channel='))
            self.assertIn('STOPPED:', text)
            self.assertIn('raw_readback=', text)

    def test_protocol_fragmentation_rejection_and_fault(self):
        with self.server(fault=True) as (connection, port, logs):
            raw = named_sequences.make_request('sequence list').SerializeToString()
            # Deliberately leave one client's frame incomplete; second remains responsive.
            connection.sendall(_VarintBytes(len(raw)) + raw[:1])
            with socket.create_connection(('127.0.0.1', port), timeout=1) as second:
                self.assertEqual(self.request(second, 'sequence status').state, 'ready')
            connection.sendall(raw[1:])
            # Read first reply without sending another request.
            length = connection.recv(1)[0]
            body = bytearray()
            while len(body) < length:
                body.extend(connection.recv(length - len(body)))
            self.assertTrue(clover_pb2.Response.FromString(body).HasField('named_sequence'))
            unsupported = clover_pb2.Request()
            unsupported.actuate_valve.valve = clover_pb2.SV001
            unsupported.actuate_valve.state = clover_pb2.POWERED_OPEN
            with self.assertRaisesRegex(ValueError, 'named_sequence only'):
                named_sequences.exchange(connection, unsupported)
            missing_name = clover_pb2.Request()
            missing_name.named_sequence.operation = clover_pb2.NamedSequenceRequest.RUN
            with self.assertRaisesRegex(ValueError, 'rejected'):
                named_sequences.exchange(connection, missing_name)
            self.assertEqual(self.request(connection, 'sequence status').state, 'ready')
            self.request(connection, 'sequence run alpha')
            info = self.wait_state(connection, 'faulted')
            self.assertNotEqual(info.error, 0)
            output = io.StringIO()
            view = ValveSequenceMode(connection, Console(file=output, force_terminal=False, width=110))
            self.assertTrue(view.execute('status'))
            self.assertIn('FAULTED', output.getvalue())
            self.assertIn('reset required', output.getvalue())
            with self.assertRaisesRegex(ValueError, 'rejected'):
                self.request(connection, 'sequence run MixedCase')
            self.request(connection, 'sequence stop')
            self.assertEqual(self.request(connection, 'sequence status').state, 'faulted')
            # Oversized/malformed frames are closed without any command execution.
            connection.sendall(_VarintBytes(257))
            self.assertEqual(connection.recv(1), b'')
            logs.seek(0)
            self.assertIn('OFF result=0', logs.read())

    def test_vlvseq_mode_and_readable_progress(self):
        with self.server() as (connection, port, logs):
            output = io.StringIO()
            console = Console(file=output, force_terminal=False, width=110)
            mode = ValveSequenceMode(connection, console, poll_interval=.01)

            def wait_output(text):
                deadline = time.monotonic() + 3
                while text not in output.getvalue():
                    if time.monotonic() > deadline:
                        self.fail(f'Missing {text!r}: {output.getvalue()}')
                    time.sleep(.01)

            actions = iter(['list', 'run missing', 'run alpha', 'status', 'stop', 'run MixedCase', 'back'])
            test = self

            class Session:
                def prompt(self, prompt):
                    test.assertIn('VLVSEQ>', prompt)
                    command = next(actions)
                    if command == 'status':
                        # Must arrive without the user manually requesting status.
                        wait_output('Alpha "test" café %')
                    elif command == 'run MixedCase':
                        wait_output('STOPPED')
                    elif command == 'back':
                        wait_output('COMPLETE')
                    return command

            # Keep stdout unchanged in this nonterminal test; production uses prompt redraw.
            with patch('valve_sequence_cli.patch_stdout', return_value=contextlib.nullcontext()):
                mode.run(Session())
            text = output.getvalue()
            self.assertIn('Stored valve sequences', text)
            self.assertIn('MixedCase', text)
            self.assertIn('Rejected:', text)
            self.assertIn('Run accepted: alpha', text)
            self.assertIn('RUNNING', text)
            self.assertIn('step 1/2', text)
            self.assertIn('Alpha "test" café %', text)
            self.assertIn('Stop requested', text)
            self.assertIn('Returned to CMD', text)
            self.assertNotIn('\x1b', text)
            self.assertTrue(mode.done.is_set())
            self.assertFalse(mode.broken)
            self.assertEqual(self.request(connection, 'sequence status').state, 'complete')

    def test_vlvseq_routing(self):
        spec = importlib.util.spec_from_file_location('client_new_test', REPO / 'scripts/client-new.py')
        client = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(client)
        with patch.object(client.ValveSequenceMode, 'run') as run:
            self.assertTrue(client.route_command('vlvseq'))
            run.assert_called_once()

    def test_request_validation(self):
        for command in ['sequence', 'sequence run', 'sequence list extra', 'sequence unknown']:
            with self.assertRaises(ValueError):
                named_sequences.make_request(command)
        request = named_sequences.make_request('sequence run MixedCase')
        self.assertEqual(request.named_sequence.name, 'MixedCase')

    def test_lost_ack_is_not_retried(self):
        class LostAck:
            writes = 0

            def sendall(self, data):
                self.writes += 1

            def recv(self, length):
                return b''

        connection = LostAck()
        with self.assertRaisesRegex(ConnectionError, 'check status'):
            named_sequences.exchange(connection, named_sequences.make_request('sequence run alpha'))
        self.assertEqual(connection.writes, 1)
