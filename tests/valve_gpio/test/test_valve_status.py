"""All mapped valves remain visible, including unavailable channels."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
import clover_pb2
from valve_status import valve_rows


class ValveStatusTest(unittest.TestCase):
    def test_readback_and_unknown_channels(self):
        info = clover_pb2.NamedSequenceInfo(state='running', powered_channels=2, known_channels=3)
        mapping = {'OFF_VALVE': 0, 'ON_VALVE': 1, 'UNAVAILABLE': 2, 'UNWIRED': None}
        self.assertEqual([row[2] for row in valve_rows(info, mapping)],
                         ['OFF', 'ON', 'UNKNOWN', 'UNASSIGNED'])

    def test_old_firmware_is_unknown(self):
        info = clover_pb2.NamedSequenceInfo(state='ready', manual_powered_channels=0)
        self.assertEqual(list(valve_rows(info, {'SV001': 0})), [('SV001', '0', 'UNKNOWN')])


class ClientValveCommandsTest(unittest.TestCase):
    def test_commands_reset_on_reconnect(self):
        from valve_status import ClientValveCommands
        tracker = ClientValveCommands()
        connection = object()
        tracker.record(connection, 1, True)
        self.assertEqual(tracker.for_connection(connection), {1: True})
        tracker.record(connection, 1, False)
        self.assertEqual(tracker.for_connection(connection), {1: False})
        self.assertEqual(tracker.for_connection(object()), {})

    def test_readback_and_command_are_independent(self):
        from unittest.mock import patch
        from valve_status import valve_table
        info = clover_pb2.NamedSequenceInfo(state='ready', powered_channels=1, known_channels=7)
        with patch('valve_status.MAP_PATH') as path:
            path.read_text.return_value = '{"EXTERNAL": 0, "CLIENT": 1, "OFF": 2, "UNWIRED": null}'
            table = valve_table(info, {1: True, 2: False})
        self.assertEqual(table.columns[2]._cells, ['ON', 'OFF', 'OFF', 'UNASSIGNED'])
        self.assertEqual(table.columns[5]._cells,
                         ['—', 'Requested ON', 'Requested OFF', '—'])


    def test_sequence_and_manual_override_columns(self):
        from unittest.mock import patch
        from valve_status import valve_table
        info = clover_pb2.NamedSequenceInfo(state='running', powered_channels=2,
                                            known_channels=3, sequence_requested_channels=1,
                                            manual_override_channels=3, manual_powered_channels=2)
        with patch('valve_status.MAP_PATH') as path:
            path.read_text.return_value = '{"MANUAL_OFF": 0, "MANUAL_ON": 1}'
            table = valve_table(info, {0: False, 1: True})
        self.assertEqual(table.columns[2]._cells, ['OFF', 'ON'])
        self.assertEqual(table.columns[3]._cells, ['ON', 'OFF'])
        self.assertEqual(table.columns[4]._cells, ['OFF', 'ON'])
