"""Only guard logic; end-to-end C++/ROS/HTTP evidence is recorded separately."""
import unittest
from mtc_webui.cpp_mock import CppMockBackend, cpp_config, normalized_namespace

class Guards(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.backend = CppMockBackend(cpp_config('/isolated', 161), clock=lambda: self.now)
        self.backend.transport = object()
        self.backend.connection_update(connected=True, action_ready=True, child_ready=True)
        for key in ('cpp_task', 'cpp_pick'):
            self.backend.observe(key, {'scope':'cpp_mock','admission_open':True,'initialized':True}, key, 100, 0)

    def test_explicit_mode_needs_no_fabricated_hardware(self):
        s = self.backend.snapshot()
        self.assertEqual(s['mode'], 'cpp_mock')
        self.assertTrue(s['operations']['submit'])
        self.assertFalse(s['robot']['available'])
        self.assertFalse(s['tool']['available'])
        self.assertFalse(s['operations']['stop'])

    def test_stale_or_replayed_readiness_never_enables_goal(self):
        self.now += 4
        self.backend.observe('cpp_task', {'scope':'cpp_mock','admission_open':True,'initialized':True}, 'cpp_task', 100, 0)
        self.assertFalse(self.backend.snapshot()['operations']['submit'])

    def test_core_lock_blocks_next_goal(self):
        self.backend.observe('cpp_pick', {'scope':'cpp_mock','admission_open':False}, 'cpp_pick', 101, 0)
        self.assertFalse(self.backend.snapshot()['operations']['submit'])

    def test_child_or_graph_conflict_blocks_goal(self):
        self.backend.connection_update(child_ready=False)
        self.assertFalse(self.backend.snapshot()['operations']['submit'])

    def test_scope_mismatch_blocks_goal(self):
        self.backend.observe('cpp_pick', {'scope':'hardware','admission_open':True}, 'cpp_pick', 101, 0)
        self.assertFalse(self.backend.snapshot()['operations']['submit'])

    def test_readonly_and_invalid_namespace(self):
        b = CppMockBackend(cpp_config('/isolated', 161, True))
        self.assertIn('当前为只读模式', b.snapshot()['operations']['reasons'])
        for value in ['', '/', '/bad-name', '../foo']:
            with self.assertRaises(ValueError): normalized_namespace(value)

if __name__ == '__main__':
    unittest.main()
