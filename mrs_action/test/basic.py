import unittest
import signal

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent))
from helpers.base_action_test import BaseActionTest
import helpers.launch_decription as ld


def generate_test_description():
    return ld.generate_test_description(
        {
            "mode": "goto",
        },
        {
            "busy_time": 3.,
            "report_duration": 1.,
            "report_interval": .1
        })


class TestBasic(BaseActionTest): 
        
    def test_basic(self):

        self._test_action_provider()

        goal_future = self._start_action_async()
        result_future = self._receive_result_async(goal_future, 10.0)
        self._test_result_success(result_future, 10.0)

        self.assertIn(2, self.fb_history) # reported waiting
        self.assertIn(3, self.fb_history) # reported flying
        

import launch_testing
@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info,
                                               allowable_exit_codes=[0, -signal.SIGINT],)
