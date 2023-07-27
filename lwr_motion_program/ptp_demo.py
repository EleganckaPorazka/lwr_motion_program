# Copyright (C) 2023 Łukasz Woliński
# You may use, distribute and modify this code under the terms of the BSD-3-Clause License.


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from math import pi

from rrlib_interfaces.action import PTP


class PTP_demo(Node):

    def __init__(self):
        super().__init__('ptp_demo')
        self._action_client = ActionClient(self, PTP, 'ptp_motion')

    def send_goal(self):
        goal_msg = PTP.Goal()
        goal_msg.start_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        goal_msg.end_position = [0.0, 0.0, 0.0, pi/2, 0.0, -pi/2, 0.0]
        goal_msg.vel_max = 0.5
        goal_msg.acc_max = 2.0
        goal_msg.dt = 0.05

        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Success: ' + str(result.success) + '\nMessage: ' + result.message)
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: ' + str(feedback.percent_complete))


def main(args=None):
    rclpy.init(args=args)

    action_client = PTP_demo()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
