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
