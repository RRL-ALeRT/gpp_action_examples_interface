#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from gpp_action_examples.action import TrajectoryToFrame
from spot_msgs.action import Trajectory

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import time


class TrajectoryToFrameAction(Node):

    def __init__(self):
        super().__init__('trajectoryToFrame')
        self._action_server = ActionServer(
            self,
            TrajectoryToFrame,
            'trajectoryToFrame',
            self.execute_callback)

        self._action_client = ActionClient(self, Trajectory, 'trajectory')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #self.timer = self.create_timer(0.1, self.on_timer)
        self.target_frame = ""
        self.rel_frame = "body"
        self.tf_available = False
        self.t = TransformStamped()

    def calc_tf(self):
        # Store frame names in variables that will be used to
        # compute transformations
        try:
            self.t = self.tf_buffer.lookup_transform(
                "frame1",
                self.rel_frame,
                rclpy.time.Time())
            self.tf_available = True
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.rel_frame} to {self.target_frame}: {ex}')
            return

    def send_goal(self, frame_id):
        goal_msg = Trajectory.Goal()
        timeout=rclpy.duration.Duration(seconds=5.0)

        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.header.frame_id = "body"
        goal_msg.target_pose.pose.position.x = self.t.transform.translation.x
        goal_msg.target_pose.pose.position.y = self.t.transform.translation.y
        goal_msg.target_pose.pose.orientation.w = self.t.transform.rotation.w

        goal_msg.duration = timeout
        goal_msg.precise_positioning = False

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.target_frame = goal_handle.request.frame_id
        while(not self.tf_available):
            self.calc_tf()

        future = self._action_client.send_goal()
        rclpy.spin_until_future_complete(self._action_client, future)
        goal_handle.succeed()

        result = TrajectoryToFrame.Result()
        return result

def main(args=None):
    rclpy.init(args=args)

    trajectoryToFrame = TrajectoryToFrameAction()

    rclpy.spin(trajectoryToFrame)


if __name__ == '__main__':
    main()
