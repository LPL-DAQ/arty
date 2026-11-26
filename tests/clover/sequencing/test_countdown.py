"""Exercise countdown key input without hardware or a real terminal."""
from contextlib import nullcontext
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

from prompt_toolkit.input import create_pipe_input
from rich.console import Console

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'client'))
from sequence_countdown import countdown
from valve_sequence_cli import ValveSequenceMode


class CountdownTest(unittest.TestCase):
    def test_terminal_input_is_closed_without_context_manager(self):
        # create_input returns an Input, unlike create_pipe_input's context manager.
        source = Mock()
        source.raw_mode.side_effect = nullcontext
        for fail in (False, True):
            with self.subTest(fail=fail), \
                    patch('sequence_countdown.sys.stdin.isatty', return_value=True), \
                    patch('sequence_countdown.create_input', return_value=source), \
                    patch('sequence_countdown.time.monotonic', side_effect=RuntimeError('test') if fail else [0, 1]):
                if fail:
                    with self.assertRaises(RuntimeError):
                        countdown(Console(file=io.StringIO()), 'alpha', seconds=0)
                else:
                    self.assertTrue(countdown(Console(file=io.StringIO()), 'alpha', seconds=0))
            source.close.assert_called_once_with()
            source.reset_mock()

    def test_cancel_keys(self):
        for key in (' ', 'q', 'Q', '\x03'):
            with self.subTest(key=key), create_pipe_input() as source:
                output = io.StringIO()
                source.send_text(key)
                self.assertFalse(countdown(Console(file=output), 'alpha', terminal_input=source))
                self.assertIn('Sequence was not started', output.getvalue())
                self.assertIn('in 10', output.getvalue())

    def test_expiry(self):
        with create_pipe_input() as source:
            self.assertTrue(countdown(Console(file=io.StringIO()), 'alpha', seconds=.01,
                                      terminal_input=source))

    def test_cancelled_run_keeps_mode_and_connection(self):
        connection = Mock()
        mode = ValveSequenceMode(connection, Console(file=io.StringIO()), poll_interval=10)
        session = Mock()
        session.prompt.side_effect = ['run alpha', KeyboardInterrupt(), 'status', 'back']
        with patch('valve_sequence_cli.sys.stdin.isatty', return_value=True), \
                patch.object(mode, 'execute', return_value=True) as execute:
            mode.run(session)
        self.assertEqual([call.args[0] for call in execute.call_args_list], ['status', 'status'])
        connection.close.assert_not_called()
        self.assertFalse(mode.broken)


class ConcurrentCountdownTest(unittest.TestCase):
    def test_manual_command_blocked_during_countdown_and_single_start(self):
        import clover_pb2
        connection = Mock()
        mode = ValveSequenceMode(connection, Console(file=io.StringIO()))
        info = clover_pb2.NamedSequenceInfo(state='ready', manual_override_channels=0)
        with patch('valve_sequence_cli.time.monotonic', return_value=100), \
                patch('valve_sequence_cli.named_sequences.exchange', return_value=info) as exchange:
            mode.schedule_run('alpha')
            self.assertFalse(mode.execute('on SV001'))
            self.assertEqual(mode.client_commands, {})
            exchange.assert_not_called()
            self.assertEqual(mode.pending_run[0], 'alpha')
            mode.advance_countdown()
            self.assertFalse(any(call.args[1].HasField('named_sequence') and
                                 call.args[1].named_sequence.operation == 1 for call in exchange.call_args_list))
            with patch('valve_sequence_cli.time.monotonic', return_value=111):
                mode.advance_countdown()
                mode.advance_countdown()
            runs = [call for call in exchange.call_args_list if call.args[1].HasField('named_sequence')
                    and call.args[1].named_sequence.operation == 1]
            self.assertEqual(len(runs), 1)
            self.assertEqual(runs[0].args[1].named_sequence.name, 'alpha')
            self.assertIsNone(mode.pending_run)

    def test_valve_menu_blocked_during_countdown_and_available_after_cancel(self):
        output = io.StringIO()
        mode = ValveSequenceMode(Mock(), Console(file=output))
        session = Mock()
        mode.schedule_run('alpha')
        mode.manual_prompt(session)
        session.prompt.assert_not_called()
        self.assertIn('MANUAL CONTROL BLOCKED', output.getvalue())
        self.assertIn('countdown', output.getvalue())
        mode.cancel_pending()
        session.prompt.return_value = 'q'
        mode.manual_prompt(session)
        session.prompt.assert_called_once()

    def test_release_is_not_sent(self):
        import clover_pb2
        mode = ValveSequenceMode(Mock(), Console(file=io.StringIO()))
        info = clover_pb2.NamedSequenceInfo(state='ready')
        with patch('valve_sequence_cli.named_sequences.exchange', return_value=info) as exchange:
            self.assertFalse(mode.execute('release SV001'))
            exchange.assert_not_called()

    def test_failed_manual_command_does_not_record_or_retry(self):
        mode = ValveSequenceMode(Mock(), Console(file=io.StringIO()))
        mode.override_supported = True
        with patch('valve_sequence_cli.named_sequences.exchange', side_effect=ConnectionError('lost ACK')) as exchange:
            self.assertFalse(mode.execute('on SV001'))
            self.assertFalse(mode.execute('on SV001'))
            self.assertEqual(exchange.call_count, 1)
            self.assertEqual(mode.client_commands, {})
            self.assertTrue(mode.broken)
