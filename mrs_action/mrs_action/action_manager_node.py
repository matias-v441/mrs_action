#!/usr/bin/env python3

import rclpy
from rclpy import Parameter
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus
from ros2service.api import get_service_class

from enum import Enum
import threading
import time

from mrs_action_msgs.action import Goto, Path, ReferenceStamped
import mrs_msgs.srv as srv
from mrs_msgs.msg import ControlManagerDiagnostics

_mode_interface = {
    "goto" : ("control_manager/goto", srv.Vec4, Goto),
    "path" : ("trajectory_generation/path", srv.PathSrv, Path),
    "reference" : ("control_manager/reference", srv.ReferenceStampedSrv, ReferenceStamped),
    }

_diag_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

UPDATE_INTERVAL = 1 #sec

class State(Enum):
    IDLE = 0,       # drone is not controlled
    REQUESTING = 1, # service request is sent
    WAITING = 2,    # drone is controlled by something else
    FLYING = 3,     # the goal is executing. 

_feedback_state = {
    State.IDLE: Goto.Feedback.STATE_IDLE,
    State.REQUESTING: Goto.Feedback.STATE_REQUESTING,
    State.WAITING: Goto.Feedback.STATE_WAITING,
    State.FLYING: Goto.Feedback.STATE_FLYING,
}

class ActionManager(Node):

    def __init__(self):
        super().__init__('action_manager')
        self.action_server_group = MutuallyExclusiveCallbackGroup()
        self.sub_group = MutuallyExclusiveCallbackGroup()

        self.mode = self.declare_parameter('mode', Parameter.Type.STRING).value
        self.get_logger().info(f"Starting {self.mode}")

        srv_name, self.srv_itf, self.action_itf = _mode_interface[self.mode]

        self._action_server = ActionServer(
            self,
            self.action_itf,
            f'~/{self.mode}',
            self.execute_callback, # ACCEPTED -> SUCCEDED/ABORTED
            goal_callback = self.goal_callback, # -> ACCEPTED
            handle_accepted_callback = self.handle_accepted_callback, # store _goal_handle for state_worker
            cancel_callback = self.cancel_callback, # CANCELING -> CANCELED
            callback_group = self.action_server_group
        )
        self._client = self.create_client(self.srv_itf, srv_name)
        self._subscription = self.create_subscription(ControlManagerDiagnostics,
                                                      "control_manager/diagnostics",
                                                      self.diagnostics_callback, 
                                                      _diag_qos,
                                                      callback_group = self.sub_group
                                                      )
        self._update_timer = self.create_timer(
            UPDATE_INTERVAL,
            self.update_callback,
            callback_group = self.action_server_group,
            autostart = False
        )

        self._state = State.IDLE
        self._goal_handle = None
        self._goal_handle_accepted = False # against accepting multiple goals
        self._client_future = None
        self._result = None
        self._have_goal = False
        self._have_goal_lock = threading.Lock()


    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new_state): # publish if updated

        if self._state == new_state: return
        old_state = self._state
        self._state = new_state

        self.get_logger().info(f"{new_state}")

        if _feedback_state[old_state] != _feedback_state[new_state]:
            feedback_msg = self.action_itf.Feedback()
            feedback_msg.state = _feedback_state[self._state]
            if self._goal_handle: 
                self._goal_handle.publish_feedback(feedback_msg)


    def diagnostics_callback(self, msg): # spins on topic | mutex subscriber group
        with self._have_goal_lock:
            self._have_goal = msg.tracker_status.have_goal


    def update_callback(self): # runs in parallel to diagnostics_callback
        with self._have_goal_lock:
            have_goal = self._have_goal

        if self._goal_handle.is_cancel_requested:
            self._cancel_goal()

        assert self._goal_handle.status == GoalStatus.STATUS_ACCEPTED

        match self.state:
            case State.IDLE:
                if not have_goal:
                    self.state = State.REQUESTING
                    self._client_future = self._send_client_request()
                else:
                    self.state = State.WAITING

            case State.REQUESTING:  
                if self._client_future.done():
                    if not self._client_future.result().success:
                        self._finish(success=False, message=self._client_future.result().message)
                    else:
                        if have_goal:
                            self.state = State.FLYING # request -> wait for diagnostics
                        else:
                            self._finish(success=True, message="already there")
                else:
                    if self._client_future.exception() and have_goal:
                        self.state = State.WAITING
                    if have_goal: # someone else took the goal while requesting (unlikely)
                        self._client_future.cancel()
                        self.state = State.WAITING
                                            
            case State.FLYING:
                if not have_goal:
                    self._finish(success=True, message="finished flying")

            case State.WAITING:
                if not have_goal:
                    self.state = State.REQUESTING
                    self._client_future = self._send_client_request()
                    

    def _send_client_request(self):
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"waiting for goto...")
        self.get_logger().info(f"Requesting goto...")

        request = self.srv_itf.Request()
        if self.action_itf is Goto:
            request.goal = self._goal_handle.request.goal
        elif self.action_itf is Path:
            request.path = self._goal_handle.request.path
        elif self.action_itf is ReferenceStamped:
            request.header = self._goal_handle.request.header
            request.reference = self._goal_handle.request.reference
        else:
            raise RuntimeError(f"Unknown action interface {self.action_itf}")

        return self._client.call_async(request)


    def _cancel_goal(self): 
        self.get_logger().info("Canceled")
        self.state = State.IDLE
        result = self.action_itf.Result()
        result.success = False
        result.message = "canceled"
        self._goal_handle.canceled(result)
        self._goal_handle = None
        self._goal_handle_accepted = False


    def _finish(self, *, success=False, message=''):
        result = self.action_itf.Result()
        result.success = success
        result.message = message
        self._result = result
        self._update_timer.cancel()
        self._goal_handle.execute() 


    def execute_callback(self, goal_handle): # mutex action group
        self.get_logger().info(f"Finished: {self._result}")
        self.state = State.IDLE
        if self._result.success:
            goal_handle.succeed(self._result)
        else:
            goal_handle.abort(self._result)
        self._goal_handle = None
        self._goal_handle_accepted = False
        return self._result


    def cancel_callback(self, cancel_request): # mutex action group
        self.get_logger().info("Cancel requested")
        if not self._goal_handle or self.state in [State.REQUESTING, State.FLYING]:
            self.get_logger().info("Cannot cancel: flight is requested")
            return CancelResponse.REJECT
        return CancelResponse.ACCEPT

    
    def goal_callback(self, goal_request): # mutex action group
        self.get_logger().info("Goal requested")
        if self._goal_handle_accepted:
            return GoalResponse.REJECT
        self._goal_handle_accepted = True
        return GoalResponse.ACCEPT
    

    def handle_accepted_callback(self, goal_handle): # mutex action group
        self.get_logger().info(f"Goal accepted: {goal_handle.request}")
        assert not self._goal_handle
        self._goal_handle = goal_handle
        if self._goal_handle.is_cancel_requested:
            self._cancel_goal()
        else:
            self._update_timer.reset()


def main(args=None):
    rclpy.init(args=args)
    action_manager = ActionManager()
    #executor = SingleThreadedExecutor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(action_manager)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        action_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 