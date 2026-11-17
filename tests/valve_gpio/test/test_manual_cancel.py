"""Cancelling either manual selection prompt must not send a command."""
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
from rich.console import Console

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from valve_sequence_cli import ValveSequenceMode


class ManualCancelTest(unittest.TestCase):
    def test_cancel_at_either_prompt(self):
        for answers in (['q'], ['back'], ['exit'], ['1', 'q'], ['1', 'back'], ['1', 'exit'],
                        [KeyboardInterrupt()], [EOFError()], ['1', KeyboardInterrupt()], ['1', EOFError()]):
            with self.subTest(answers=answers):
                connection = Mock()
                mode = ValveSequenceMode(connection, Console(file=io.StringIO()))
                session = Mock()
                session.prompt.side_effect = answers
                with patch.object(mode, 'execute') as execute:
                    mode.manual_prompt(session)
                execute.assert_not_called()
                self.assertEqual(connection.mock_calls, [])

    def test_release_selector_and_all(self):
        for choice, command in [('1', 'release SV001'), ('all', 'release all')]:
            mode = ValveSequenceMode(Mock(), Console(file=io.StringIO()))
            session = Mock()
            session.prompt.return_value = choice
            with patch.object(mode, 'execute') as execute:
                mode.release_prompt(session)
            execute.assert_called_once_with(command)

    def test_cancel_release_sends_nothing(self):
        for choice in ['q', 'back', 'exit', KeyboardInterrupt(), EOFError()]:
            mode = ValveSequenceMode(Mock(), Console(file=io.StringIO()))
            session = Mock()
            session.prompt.side_effect = [choice]
            with patch.object(mode, 'execute') as execute:
                mode.release_prompt(session)
            execute.assert_not_called()

    def test_release_all_includes_unmapped_channels(self):
        import clover_pb2
        mode = ValveSequenceMode(Mock(), Console(file=io.StringIO()), client_commands={0: True, 31: False})
        info = clover_pb2.NamedSequenceInfo(state='running', manual_override_channels=(1 << 31) | 1)
        with patch('valve_sequence_cli.named_sequences.exchange', return_value=info) as exchange:
            self.assertTrue(mode.execute('release all'))
        requests = [call.args[1] for call in exchange.call_args_list]
        self.assertEqual([req.manual_valve.channel for req in requests[1:]], [0, 31])
        self.assertTrue(all(req.manual_valve.release for req in requests[1:]))
        self.assertEqual(mode.client_commands, {})

    def test_release_all_stops_on_lost_ack(self):
        import clover_pb2
        mode = ValveSequenceMode(Mock(), Console(file=io.StringIO()))
        info = clover_pb2.NamedSequenceInfo(state='ready', manual_override_channels=3)
        with patch('valve_sequence_cli.named_sequences.exchange', side_effect=[info, ConnectionError('lost ACK')]) as exchange:
            self.assertFalse(mode.execute('release all'))
            self.assertEqual(exchange.call_count, 2)
            self.assertTrue(mode.broken)


    def test_r_then_all_routes_to_release_selector(self):
        mode = ValveSequenceMode(Mock(), Console(file=io.StringIO()), poll_interval=10)
        session = Mock()
        session.prompt.side_effect = ['r', 'all', 'back']
        with patch.object(mode, 'execute', return_value=True) as execute:
            mode.run(session)
        self.assertEqual([call.args[0] for call in execute.call_args_list],
                         ['status', 'release all'])


    def test_direct_shortcuts_are_not_routed(self):
        mode = ValveSequenceMode(Mock(), Console(file=io.StringIO()), poll_interval=10)
        session = Mock()
        session.prompt.side_effect = ['on SV001', 'off SV001', 'release SV001', 'release all', 'cancel', 'back']
        with patch.object(mode, 'execute', return_value=True) as execute:
            mode.run(session)
        self.assertEqual([call.args[0] for call in execute.call_args_list], ['status'])
