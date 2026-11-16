"""Manual prompt cancellation must not touch the board connection."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

SCRIPTS = Path(__file__).resolve().parents[3] / 'scripts'
sys.path.insert(0, str(SCRIPTS))


class ManualCancelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('manual_cancel_client', SCRIPTS / 'client-new.py')
        cls.client = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.client)

    def test_cancel_at_either_prompt(self):
        for answers in (['q'], ['back'], ['exit'], ['1', 'q'], ['1', 'back'], ['1', 'exit'],
                        [KeyboardInterrupt()], [EOFError()], ['1', KeyboardInterrupt()], ['1', EOFError()]):
            with self.subTest(answers=answers):
                connection = Mock()
                with patch.object(self.client, 'sock', connection), \
                        patch.object(self.client, 'console'), \
                        patch.object(self.client.Prompt, 'ask', side_effect=answers), \
                        patch.object(self.client.named_sequences, 'exchange') as exchange:
                    self.client.cmd_actuate_valve()
                exchange.assert_not_called()
                self.assertEqual(connection.mock_calls, [])
