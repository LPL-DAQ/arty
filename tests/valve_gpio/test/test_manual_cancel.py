"""Cancelling either manual selection prompt must not send a command."""
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
from rich.console import Console

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'client'))
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

    def test_direct_shortcuts_are_not_routed(self):
        mode = ValveSequenceMode(Mock(), Console(file=io.StringIO()), poll_interval=10)
        session = Mock()
        session.prompt.side_effect = ['on SV001', 'off SV001', 'r', 'release SV001', 'release all', 'cancel', 'back']
        with patch.object(mode, 'execute', return_value=True) as execute:
            mode.run(session)
        self.assertEqual([call.args[0] for call in execute.call_args_list], ['status'])
