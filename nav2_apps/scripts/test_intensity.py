#! /usr/bin/env python3
# Copyright Jēkabs Jaunslavietis
# ROS2 Masterclass, Checkpoint 6
import threading
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Twist, Polygon, Point32
from std_msgs.msg import Empty
from lifecycle_msgs.srv import ChangeState
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf_transformations  import euler_from_quaternion, quaternion_from_euler

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import tf2_ros

import enum, math


class ApproachTable(Node):
    def __init__(self):
        super().__init__('approach_table')
        
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_sub_clb, 3)

        self.get_logger().info("approach_table created")
    
    def laser_sub_clb(self, msg):

        self.laser_scan_data = msg
        print(msg.intensities)

    


def main():
    rclpy.init()

    try:

        approach_table = ApproachTable()
        

        exe = rclpy.executors.MultiThreadedExecutor()

        exe.add_node(approach_table)
        exe.spin()
    finally:
        exe.shutdown()

        rclpy.shutdown()

if __name__=='__main__':
    main()
