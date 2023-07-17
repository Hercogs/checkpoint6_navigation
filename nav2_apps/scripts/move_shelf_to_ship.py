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
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf_transformations  import quaternion_from_euler

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import tf2_ros

import enum, math

lock = threading.Lock()

class RobotState(enum.Enum):
    INITIALIZED = 0
    POSITION_LOADING = 1
    POSITION_UNDER_TABLE = 2
    POSITION_SHIPPING = 3
    POSITION_FINAL = 4
    ERROR = 5

class NavigateRobot(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.positions = {
                    'init_position':        [0.0, 0.0, 0.0],
                    'loading_position':     [4.45, -0.5, -1.57],
                    'shipping_position':    [0.95, -2.6, 3.14]}

        self.navigator = BasicNavigator()

        # Wait for navigation system to fully activate
        self.get_logger().info("Waiting for NAV2 to work")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("NAV2 is ready to work")

        # Set initial position
        self.set_initial_position()

        self.state = RobotState.INITIALIZED

        self.timer = self.create_timer(1, self.timer_clb, callback_group=ReentrantCallbackGroup())

        # Approach client
        self.approach_table_cli = self.create_client(Trigger, '/approach_shelf')

        # Publishers, subscribers
        self.pub_lift_table = self.create_publisher(Empty, '/elevator_up', 3)
        self.pub_down_table = self.create_publisher(Empty, '/elevator_down', 3)
        self.pub_global_footprint = self.create_publisher(Polygon, '/global_costmap/footprint', 3)
        self.pub_local_footprint = self.create_publisher(Polygon, '/local_costmap/footprint', 3)
        self.speed_pub = self.create_publisher(Twist, 'robot/cmd_vel', 1)
    
    
    def set_initial_position(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.positions['init_position'][0]
        initial_pose.pose.position.y = self.positions['init_position'][1]

        q = quaternion_from_euler(0, 0, self.positions['init_position'][2])

        initial_pose.pose.orientation.x = q[0]
        initial_pose.pose.orientation.y = q[1]
        initial_pose.pose.orientation.z = q[2]
        initial_pose.pose.orientation.w = q[3]

        self.navigator.setInitialPose(initial_pose)

    # As state machine
    def timer_clb(self) -> None:
        self.timer.cancel()
        print('Robot state: ' + str(self.state))

        if (self.state == RobotState.INITIALIZED):
            self.go_to_position(position="loading_position")
        elif (self.state == RobotState.POSITION_LOADING):
            self.approach_table()
        elif (self.state == RobotState.POSITION_UNDER_TABLE):
            self.go_to_position(position="shipping_position")
        elif (self.state == RobotState.POSITION_SHIPPING):
            # Put down table

            ## NOT working
            #self.pub_down_table.publish(Empty())

            # CHange robot footprint size
            msg = Polygon()

            p1 = Point32()
            p2 = Point32()
            p3 = Point32()
            p4 = Point32()
            msg.points = [p1, p2, p3, p4]

            p1.x = 0.25
            p1.y = 0.25
            p2.x = 0.25
            p2.y = -0.25
            p3.x = -0.25
            p3.y = -0.25
            p4.x = -0.25
            p4.y = 0.25
            self.pub_local_footprint.publish(msg)
            self.pub_global_footprint.publish(msg)

            self.create_rate(0.5).sleep()

            # Move robot bit to back
            time = 0
            msg = Twist()
            while(time < 3.5):
                msg.linear.x = -0.25
                msg.angular.z = 0.0
                self.speed_pub.publish(msg)
                self.create_rate(10).sleep()
                time +=0.1

            msg.linear.x = -0.0
            msg.angular.z = 0.0
            self.speed_pub.publish(msg)

            # Go back init position
            self.go_to_position(position="init_position")
        elif (self.state == RobotState.POSITION_FINAL):
            self.get_logger().info("Mision completed!!! \n\n"\
                        "Please exit the program")
        elif (self.state == RobotState.ERROR):
            self.get_logger().error("Got error! Restart node")
        else:
            pass
    

    def approach_table(self):
        if (not self.approach_table_cli.wait_for_service(timeout_sec=1.0)):
            self.get_logger().error("Service approach table not avialable")
            self.state = RobotState.ERROR
            self.timer.reset()
            return
        req = Trigger.Request()
        future = self.approach_table_cli.call_async(req)
        #rclpy.spin_until_future_complete(self, future, timeout_sec=60)
        while(not future.done()):
            self.create_rate(1.0).sleep()

        status = future.result().success
        if status != True:
            self.get_logger().error("Failed to get robot under table")
            self.state == RobotState.ERROR
            return
        else:
            self.get_logger().info("Robot is under table")

            # Lift table
            ## Not working
            #self.pub_lift_table.publish(Empty())

            # Change footprint
            msg = Polygon()

            p1 = Point32()
            p2 = Point32()
            p3 = Point32()
            p4 = Point32()
            msg.points = [p1, p2, p3, p4]

            p1.x = 0.45
            p1.y = 0.39
            p2.x = 0.45
            p2.y = -0.39
            p3.x = -0.45
            p3.y = -0.39
            p4.x = -0.45
            p4.y = 0.39
            self.pub_local_footprint.publish(msg)
            self.pub_global_footprint.publish(msg)

            self.create_rate(1.0).sleep()

            self.navigator.clearAllCostmaps()
     
        self.state = RobotState(self.state.value + 1)
        self.create_rate(0.5).sleep()

        self.get_logger().warn(f'Because of cannot lift table. robot moves backward and replan track')

        time = 0
        msg = Twist()
        while(time < 10):
            msg.linear.x = -0.13
            msg.angular.z = 0.0
            if time > 5.5:
                msg.angular.z = 0.1
            if time > 7.5:
                msg.angular.z = -0.09
            self.speed_pub.publish(msg)
            self.create_rate(10).sleep()
            time +=0.1
        self.get_logger().info(f'Finished going backwards')

        self.timer.reset()
    
    def go_to_position(self, position):
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        target_pose.pose.position.x = self.positions[position][0]
        target_pose.pose.position.y = self.positions[position][1]

        q = quaternion_from_euler(0, 0, self.positions[position][2])

        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        res = self.navigator.goToPose(target_pose)

        if not res:
            self.get_logger().error("goToPose(): failed")

        self.get_logger().info("goToPose(): ok")

        i = 0
        # Move to timer
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if (feedback and i % 10 == 0):
                print('Estimated time of arrival at ' + position +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
        
        result = self.navigator.getResult()
        if (result == TaskResult.SUCCEEDED):
            self.get_logger().info("SUCEEDED")
            self.state = RobotState(self.state.value + 1)
            #self.state = RobotState.POSITION_LOADING
        else:
            self.get_logger().info(f"Failed {result}")
            self.state = RobotState.ERROR
        self.timer.reset()


class ApproachTable(Node):
    def __init__(self):
        super().__init__('approach_table')

        # Create service for gettign under the table
        self.approach_table_srv = self.create_service(Trigger, '/approach_shelf',
                                                        self.approach_table_srv_clb,
                                                        callback_group=ReentrantCallbackGroup())
        
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_sub_clb, 3)

        # Create timer and stop it
        self.timer_tf_publisher = self.create_timer(0.1, self.timer_tf_publisher_clb)
        self.timer_tf_publisher.cancel()

        # Create TF broadcaster
        self.br = tf2_ros.TransformBroadcaster(self)

        # Create tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create Twist publisher
        self.speed_pub = self.create_publisher(Twist, 'robot/cmd_vel', 1)

        self.laser_scan_data = None

        self.get_logger().info("approach_table created")
    
    def laser_sub_clb(self, msg):
        lock.acquire()
        self.laser_scan_data = msg
        lock.release()
    
    def approach_table_srv_clb(self, req, res):
        self.timer_tf_publisher.reset()

        # Let the TF publisher start
        self.create_rate(0.5).sleep()

        # Try to find transform
        target_frame = 'robot_base_link'
        ref_frame = 'cart_frame'
    
        scale_forward_speed = 0.1
        scale_rotation = 0.8

        distance = 99.0
        angle = 0.0

        msg = Twist()

        while(distance > 0.47):
            try:
                t = self.tf_buffer.lookup_transform(
                    target_frame,
                    ref_frame,
                    rclpy.time.Time())
            except tf2_ros.TransformException as ex:
                self.get_logger().info(
                                f'Could not transform {target_frame} to {ref_frame}: {ex}')
                self.timer_tf_publisher.cancel()
                res.success = False
                return res
        
            distance = math.sqrt(
                        t.transform.translation.x ** 2 +
                        t.transform.translation.y ** 2)
            
            angle = math.atan2(
                        t.transform.translation.y,
                        t.transform.translation.x)
            
            msg.linear.x = scale_forward_speed * distance + 0.05
            msg.angular.z = scale_rotation * angle + (0.05 * angle/abs(angle))

            self.speed_pub.publish(msg)
            self.create_rate(15).sleep()
            print(f'Dst: {distance}')
            
        self.get_logger().info(f'Finished moving to TF')
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.speed_pub.publish(msg)
        self.create_rate(1.0).sleep()


        time = 0
        while(time < 0.2):
            # Now move right a bit
            msg.linear.x = 0.0
            msg.angular.z = -0.1
            self.speed_pub.publish(msg)
            self.create_rate(10).sleep()
            time +=0.1
        self.get_logger().info(f'Finished turning')
        
        
        # # Now move straig a bit
        time = 0
        while(time < 6.5):
            msg.linear.x = 0.115
            msg.angular.z = 0.0
            self.speed_pub.publish(msg)
            self.create_rate(10).sleep()
            time +=0.1
        self.get_logger().info(f'Finished going froward')

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.speed_pub.publish(msg)

        self.timer_tf_publisher.cancel()
        res.success = True
        return res

    

    def timer_tf_publisher_clb(self) -> None:

        leg_index_list = []
        self.get_shelf_pos(leg_index_list)

        #print('List len: ', len(leg_index_list), leg_index_list)

        # TF broadcaster
        transfor_msg = TransformStamped()
        # transfor_msg.header.stamp = self.get_clock().now().to_msg()
        transfor_msg.header.stamp = self.laser_scan_data.header.stamp
        transfor_msg.header.frame_id = 'robot_front_laser_link'
        transfor_msg.child_frame_id = 'cart_frame'

        if len(leg_index_list) != 2:
            #self.get_logger().info("Leg cnt wrong %d" % len(leg_index_list))
            return
        
        angle1 = -135 + leg_index_list[0] * 0.25
        angle2 = -135 + leg_index_list[1] * 0.25

        radius1 = self.laser_scan_data.ranges[int(leg_index_list[0])]
        radius2 = self.laser_scan_data.ranges[int(leg_index_list[1])]

        p1 = Point()
        p2 = Point()

        p1.x = radius1 * math.cos(math.radians(angle1))
        p1.y = radius1 * math.sin(math.radians(angle1))
        p2.x = radius2 * math.cos(math.radians(angle2))
        p2.y = radius2 * math.sin(math.radians(angle2))
        
        # Find middle point
        transfor_msg.transform.translation.x = (p1.x + p2.x) / 2
        transfor_msg.transform.translation.y = (p1.y + p2.y) / 2
        transfor_msg.transform.translation.z = 0.0

        transfor_msg.transform.rotation.x = 0.0
        transfor_msg.transform.rotation.y = 0.0
        transfor_msg.transform.rotation.z = 0.0
        transfor_msg.transform.rotation.w = 1.0

        self.br.sendTransform(transfor_msg)


    def get_shelf_pos(self, list_data):
        lock.acquire()

        leg_len = 0

        intenisty_threshold = 3000
        for cnt, x in enumerate(self.laser_scan_data.intensities):
            if x > intenisty_threshold:
                leg_len += 1
            elif leg_len > 1:
                list_data.append(cnt - (leg_len/2) - 1)
                leg_len = 0
            else:
                leg_len = 0

        lock.release()



def main():
    rclpy.init()

    try:
        navigate_robot = NavigateRobot()
        approach_table = ApproachTable()
        

        exe = rclpy.executors.MultiThreadedExecutor()
        exe.add_node(navigate_robot)
        exe.add_node(approach_table)
        exe.spin()
    finally:
        exe.shutdown()
        navigate_robot.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
