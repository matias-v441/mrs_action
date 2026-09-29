#!/usr/bin/env python3

import time
from threading import Thread, Lock

import rclpy
from rclpy import Parameter
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import mrs_msgs.srv as srv
from mrs_msgs.msg import ControlManagerDiagnostics


class FakeGoto(Node):

    def __init__(self):
        super().__init__("fake_goto")

        # default | fail
        self.mode = self.declare_parameter('mode', "").value

        self.busy_time = self.declare_parameter('busy_time', 0.).value
        self.report_delay = self.declare_parameter('report_delay', 0.).value
        self.report_duration = self.declare_parameter('report_duration', 1.).value
        self.report_interval = self.declare_parameter('report_interval', .1).value

        self.flying = False
        self.goals = []

        self.service_ = self.create_service(
            srv.Vec4,
            'control_manager/goto',
            self._handle_request
        )
                
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher_ = self.create_publisher(
            ControlManagerDiagnostics,
            'control_manager/diagnostics',
            qos
        )

        self.flying_thread = None
        self.flying_lock = Lock()

        self.busy_thread = None
        if self.busy_time != 0.:
            self.flying = True
            def busy():
                self.get_logger().info(f"Start busy for {self.busy_time}")
                time.sleep(self.busy_time)
                self.get_logger().info("End busy")
                with self.flying_lock:
                    self.flying = False
            self.busy_thread = Thread(target=busy, daemon=True)
            self.busy_thread.start()

        self.publish_timer = self.create_timer(
            self.report_interval,
            self._publish_flying
        )
        self.get_logger().info("Started FakeGoto")


    def _publish_flying(self):
        msg = ControlManagerDiagnostics()
        with self.flying_lock:
            msg.tracker_status.have_goal = self.flying
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published {msg.tracker_status.have_goal}")


    def _fly(self): 
        time.sleep(self.report_delay)
        self.get_logger().info(f"Started flying")
        with self.flying_lock:
            self.flying = True
        time.sleep(self.report_duration)
        with self.flying_lock:
            self.flying = False
            self.get_logger().info(f"Finished flying")


    def _handle_request(self, request, response):
        self.get_logger().info(str(response))
        self.get_logger().info(f"Received request: {request}")
        if self.mode == "fail":
            response.success = False
            response.message = "No. I will not fly."
            return response
        response.success = True
        start_flying = False
        with self.flying_lock:
            if not self.flying:
                response.message = "Start flying"
                start_flying = True
                self.goals = []
            else:
                response.message = "Already flying"
        self.goals.append(request.goal)
        self.get_logger().info(f"Goals: {self.goals}")
        if start_flying:
            if self.flying_thread and self.flying_thread.is_alive():
                self.flying_thread.join()
            self.flying_thread = Thread(target=self._fly, daemon=True)
            self.flying_thread.start()
        return response


def main(args=None):
    rclpy.init()
    node = FakeGoto()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()