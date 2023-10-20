# Copyright (C) 2023 Łukasz Woliński
# You may use, distribute and modify this code under the terms of the BSD-3-Clause License.


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi
from rcl_interfaces.msg import ParameterDescriptor

from rrlib_interfaces.action import CART


class LIN_demo(Node):

    def __init__(self):
        super().__init__('lin_demo')
        
        param_descriptor = ParameterDescriptor(description='Time step.')
        
        self.declare_parameter('dt', 0.01, param_descriptor)
        
        self.action_client_ = ActionClient(self, CART, 'cartesian_motion')
        
        self.joint_publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        self.joint_point_subscriber_ = self.create_subscription(JointTrajectoryPoint, 'ik_jnt_traj', self.listener_callback, 10)
        self.joint_point_subscriber_  # prevent unused variable warning
        
        self.result_ = False

    def send_goal(self, goal_msg):
        self.action_client_.wait_for_server()
        
        self.send_goal_future_ = self.action_client_.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self.send_goal_future_.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return

        self.get_logger().info("Goal accepted.")

        self.get_result_future_ = goal_handle.get_result_async()
        self.get_result_future_.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Success: " + str(result.success))
        self.get_logger().info("Message: " + result.message)
        self.result_ = result.success
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("Received feedback: " + str(feedback.percent_complete))
    
    def listener_callback(self, msg):
        q = msg.positions
        dqdt = msg.velocities
        
        response = JointState()
        now = self.get_clock().now()
        response.header.stamp = now.to_msg()
        response.name = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6', 'Joint_7']
        response.position = q
        self.joint_publisher_.publish(response)
        
    def motion_program(self):
        start_jnt = JointState()
        now = self.get_clock().now()
        start_jnt.header.stamp = now.to_msg()
        start_jnt.name = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5', 'Joint_6', 'Joint_7']
        start_jnt.position = [pi/4.0, 0.0, 0.0, -pi/2.0, 0.0, pi/2.0, 0.0]
        self.joint_publisher_.publish(start_jnt)
        
        goal_msg = CART.Goal()
        goal_msg.motion_type = "LIN"
        goal_msg.start_pose = [-0.275772, -0.275772, 0.6325, 0.382683, -0.92388, 0.0, 0.0]
        goal_msg.end_pose = [0.0, 0.39, 0.6325, -0.707107, -0.707107, 0.0, 0.0]
        goal_msg.vel_max = 0.5
        goal_msg.acc_max = 2.0
        dt_param = self.get_parameter('dt')
        goal_msg.dt = dt_param.value
        self.send_goal(goal_msg)
        
        # ~ while (self.result_ is False):
            # ~ pass
        
        # ~ self.get_logger().info(str(self.result_))


def main(args=None):
    rclpy.init(args=args)

    action_client = LIN_demo()
    
    action_client.motion_program()
    
    # if goal completed, then move to another goal
    # maybe add a subscriber for JointState to read the new starting position
    #goal_msg.start_position = [pi/4, 0.0, 0.0, pi/2, 0.0, -pi/2, pi/4]
    #goal_msg.end_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #action_client.send_goal(goal_msg)
    #https://github.com/HotBlackRobotics/hotblackrobotics.github.io/blob/master/en/blog/_posts/2018-01-29-seq-goals-py.md
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
