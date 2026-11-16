"""Exercise countdown key input without hardware or a real terminal."""
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

from prompt_toolkit.input import create_pipe_input
from rich.console import Console

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from sequence_countdown import countdown
from valve_sequence_cli import ValveSequenceMode


class CountdownTest(unittest.TestCase):
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
        session.prompt.side_effect = ['run alpha', 'status', 'back']
        with patch('valve_sequence_cli.countdown', return_value=False), \
                patch.object(mode, 'execute', return_value=True) as execute:
            mode.run(session)
        self.assertEqual([call.args[0] for call in execute.call_args_list], ['status', 'status'])
        connection.close.assert_not_called()
        self.assertFalse(mode.broken)
