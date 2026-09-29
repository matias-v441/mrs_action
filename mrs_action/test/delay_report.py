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
            "update_interval": .1
        },
        {
            "report_delay": 1.,
            "report_duration": 3.,
            "report_interval": .1
        })


class TestDelayReport(BaseActionTest): 
        
    def test_delay_report(self):

        self._test_action_provider()

        goal_future = self._start_action_async()
        result_future = self._receive_result_async(goal_future, 10.0)
        self._test_result_success(result_future, 10.0)

        self.assertIn(3, self.fb_history) # flight should be observed (action must finish after)
