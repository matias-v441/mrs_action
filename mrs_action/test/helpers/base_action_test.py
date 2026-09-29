import unittest
import time

import rclpy
from rclpy.node import Node 
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from mrs_action_msgs.action import Goto, Path, ReferenceStamped

class BaseActionTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()


    def setUp(self):
        self.node = rclpy.create_node('test_goto')
        self.client = ActionClient(self.node, Goto, '/test/action_manager/goto')
        self.fb_history = None

    def tearDown(self):
        self.node.destroy_node()

    def _test_action_provider(self):
        """Verify there is only one action server provider on the action name"""

        self.node.get_logger().info("Waiting for action server...")
        self.assertTrue(self.client.wait_for_server(timeout_sec=10.), 'Action service is unavailable')

        rclpy.spin_once(self.node, timeout_sec=0.5)
        providers = self._find_action_servers().get("/test/action_manager/goto", [])
        self.node.get_logger().info(f"Providers: {providers}")
        self.assertEqual(len(providers),1,
                         "There has to be only one action provider for /test/action_manager/")


    def _find_action_servers(self):
        providers_by_action = {}

        ACTION_SERVICE_SUFFIXES = (
            "/_action/send_goal",
            "/_action/get_result",
            "/_action/cancel_goal",
        )

        for node_name, node_ns in self.node.get_node_names_and_namespaces():
            try:
                services = self.node.get_service_names_and_types_by_node(node_name, node_ns)
            except Exception as e:
                self.node.get_logger().warn(
                    f"Could not inspect {node_ns}/{node_name}: {e}"
                )
                continue

            found = {}

            for srv_name, srv_types in services:
                for suffix in ACTION_SERVICE_SUFFIXES:
                    if srv_name.endswith(suffix):
                        action_name = srv_name[:-len(suffix)]
                        found.setdefault(action_name, set()).add(suffix)

            for action_name, suffixes in found.items():
                if set(ACTION_SERVICE_SUFFIXES).issubset(suffixes):
                    providers_by_action.setdefault(action_name, []).append(
                        (node_name, node_ns)
                    )

        return providers_by_action


    def _start_action_async(self, *, goal_val=0.):

        self.fb_history = []
        def feedback_cb(msg):
                self.node.get_logger().info(f"Feedback: {msg.feedback.state}")
                self.fb_history.append(msg.feedback.state)

        self.node.get_logger().info("Goal sent")

        request = Goto.Goal()
        request.goal = [goal_val]*4
        return self.client.send_goal_async(request, feedback_callback=feedback_cb)


    def _receive_result_async(self, goal_future, timeout, fail=False):
        end_time = time.time() + timeout
        while not goal_future.done() and time.time() < end_time:
            self.node.get_logger().info("Waiting for goal to be received...")
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(goal_future.done(), 'Timed out waiting for goal handle')
        self.node.get_logger().info("Goal handle received")
        goal_handle = goal_future.result()
        self.assertIsNotNone(goal_handle)
        if fail:
            self.assertFalse(goal_handle.accepted, "Goal is supposed to be rejected")
            return None
        self.assertTrue(goal_handle.accepted, "Goal handle has been rejected unexpectedly")
        return goal_handle.get_result_async()


    def _test_result_success(self, result_future, timeout, *, expect_success=True):

        end_time = time.time() + timeout
        while not result_future.done() and time.time() < end_time:
            self.node.get_logger().info("Waiting for goal to complete...")
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(result_future.done(), 'Timed out waiting for action result')

        wrap_result = result_future.result()
        self.assertIsNotNone(wrap_result)
        self.node.get_logger().info(f"Result {wrap_result.result}")
        self.assertTrue(wrap_result.result.success == expect_success)

