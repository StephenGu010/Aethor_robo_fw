"""Check real task dispatch ordering for shaft receipts in both experimental build profiles.

This is a source-order contract, not physical timing evidence; ARM builds exercise each profile.
"""
from pathlib import Path
import re
import unittest
ROOT = Path(__file__).resolve().parents[2]

class ShaftTaskReceiptContract(unittest.TestCase):
    """Require post-collection observation before either timestamped receipt dispatch path."""
    def test_upper_bound_after_collection(self):
        """Old pre-collect task clocks must never masquerade as send-completion observation upper bounds."""
        source = (ROOT / 'Core/Src/freertos.c').read_text()
        start = source.index('AdrcCanReceipt adrcReceipt = stm32_adrc_channel_collect();')
        end = source.index('aethor_app_integrated_set_can_idle', start)
        block = source[start:end]
        clock = block.find('completedObservedUs = AethorMonotonicTimestampUs();')
        self.assertGreater(clock, 0, 'No fresh observation sampled after actual hardware collect')
        calls = list(re.finditer(r'aethor_app_adrc_report_transmit_at\([^;]+;', block))
        self.assertEqual(len(calls), 2, 'Bench and integrated profiles must each dispatch timed proof')
        for call in calls:
            self.assertGreater(call.start(), clock)
            self.assertRegex(call.group(), r',\s*completedObservedUs\);$')
        self.assertNotIn('aethor_app_adrc_report_transmit(', block)
        stop_service = source.index('aethor_app_service(timestampUs)', start)
        self.assertLess(start + calls[-1].end(), stop_service)

    def test_formatter_serialization(self):
        """Both application profiles hold the same task critical boundary for the entire bridge formatter call."""
        source = (ROOT / 'App/aethor_app.c').read_text()
        integrated = source[source.index('static ProtocolEngineStatus aethor_app_integrated_process_line('):]
        integrated = integrated[:integrated.index('    return status;\n}')] 
        self.assertLess(integrated.index('aethor_app_enter_task_critical();'), integrated.index('adrc_app_bridge_process_line('))
        self.assertGreater(integrated.rindex('aethor_app_exit_task_critical();'), integrated.rindex('adrc_app_bridge_process_line('))
        public = source[source.index('ProtocolEngineStatus aethor_app_process_protocol_line('):]
        public = public[:public.index('\n}\n')]
        bench = public[public.index('#else\n    {'):] 
        self.assertLess(bench.index('aethor_app_enter_task_critical();'), bench.index('adrc_app_bridge_process_line('))
        self.assertGreater(bench.index('aethor_app_exit_task_critical();'), bench.index('adrc_app_bridge_process_line('))
        adapter = (ROOT / 'App/Adrc/adrc_shaft_pilot_adapter.c').read_text()
        self.assertTrue('static ShaftPilotTrace trace_readback;' in adapter, 'Missing serialized static snapshot')
        self.assertTrue('ShaftPilotTrace *record = &trace_readback;' in adapter, 'Exporter still uses task stack snapshot')

if __name__ == '__main__':
    unittest.main()
