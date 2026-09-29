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
            "mode": "goto"
        },
        {
            "report_delay": 0.,
            "report_duration": 3.,
            "report_interval": .5
        })


class TestMultipleGoals(BaseActionTest): 
        
    def test_multiple_goals(self):

        self._test_action_provider()

        goal_future = self._start_action_async(goal_val=0.)
        result_future = self._receive_result_async(goal_future, 10.0)

        # reject subsequent goals until the current one is done
        next_goal_future = self._start_action_async(goal_val=1.)
        self._receive_result_async(next_goal_future, 10.0, fail=True)
        
        self._test_result_success(result_future, 10.0)

        # once done - execute the next goal
        for i in range(2,5):
            self.node.get_logger().info(f"Sending next goal {i}")
            next_goal_future = self._start_action_async(goal_val=i)
            result_future = self._receive_result_async(next_goal_future, 10.0)
            self._test_result_success(result_future, 10.0)
        