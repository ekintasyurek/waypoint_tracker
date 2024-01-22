#!/usr/bin/env python3
# -*- coding: utf-8 -*-
## EB
## waypoint_follower.py
## 
## YZV406E Assignment 2 Skeleton
## 
## Notes to consier: Few helper functions and code snippets are already given to you. Examine the code carefully beforehand.
##
## If you want to make use of the map, use occupancy_grid variable.
##
## 
## STUDENT_ID:<150190108>
import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Twist, PoseArray, PoseStamped, Pose
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import math
from enum import Enum
"""
HELPER FUNCTIONS
"""
class STATE(Enum):
    MOVE_TO_GOAL = 'move_to_goal'
    FOLLOW_WALL = 'follow_wall'
    REACHED = 'reached'

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def angle_between(x, x0, y, y0):
    angle = math.atan2(y - y0, x - x0)
    return angle

def distance_between(x, x0, y, y0):
    distance = math.sqrt((x0 - x)**2 + (y0 - y)**2)
    return distance

class Navigator(Node):
    """
    Navigator node to make robot go from location A to B. 
    [IMPORTANT]
    IMPLEMENT YOUR CODES WITHIN THIS CLASS (You can define helper functions outside the class if you want)
    [IMPORTANT]
    """
    def __init__(self):
        super().__init__('waypoint_follower')
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.sub_route1 = self.create_subscription(
            PoseArray,
            '/route1',
            self.set_route1,
            10,
        )# subscribe first route
        self.sub_route2 = self.create_subscription(
            PoseArray,
            '/route2',
            self.set_route2,
            10
        ) # subscribe second route
        self.subscription_waypoint = self.create_subscription(
            PoseStamped,
            '/waypoint',
            self.waypoint_callback,
            10) # subscribe next waypoint
        self.publish_twist = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.cli = self.create_client(GetMap, '/map_server/map')
        #/map_server/map
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetMap.Request()
        
        ### MAP ###
        self.occupancy_grid = [] # OCCUPANCY GRID MESSAGE VARIABLE. You can also use this knowledge
                                # to handle obstacle.
        self.map_origin = Pose() # origin as a Pose type.
        self.map_width = 0 # width
        self.map_height = 0 # height
        self.map_resolution = 0 # size of each grid cell
        ###     ###
        self.tf_buffer = Buffer() # for transformation
        self.tf_listener = TransformListener(self.tf_buffer, self) # for transformation
        self.goal_dist_thld = 0.2 # max acceptable distance between robot and the goal
        self.angle_err_thld = 0.5-0.5*math.cos(math.pi/4) # max acceptable angle error between robot and the goal
        self.is_route1_set= 0 # 0 or 1, if 1 route1 is acquired by the topic
        self.is_route2_set = 0 # 0 or 1, if 1 route2 is acquired by the topic
        self.route1 = PoseArray() # route1
        self.route2 = PoseArray() # route2
        self.waypoint = PoseStamped() # next wayoint
        self.prev_distance = 0
        self.chatty_map = False # you may set chatty_map to true if you want to see map on the terminal
        
        self.state = STATE.MOVE_TO_GOAL
        self.entry_point = [0,0]
        self.closest_point = [0,0]
        self.estimated_distance = 0.0
        self.go_to_closest_point = False
        self.flag = True

    def send_request(self):
        self.future = self.cli.call_async(self.req)

    def set_route1(self, msg):
        if (self.is_route1_set == 0):
            self.route1 = msg
            self.is_route1_set = 1
        else:
            pass

    def set_route2(self, msg):
        if (self.is_route2_set == 0):
            self.route2 = msg
            self.is_route2_set = 1
        else:
            pass

    def waypoint_callback(self,msg):
        self.waypoint = msg

    def scan_callback(self, msg):
        if self.flag:
            return

        velocity_vec = Twist()
        
        x = self.waypoint.pose.position.x
        x0 = robotX_tf
        y = self.waypoint.pose.position.y
        y0 = robotY_tf

        self.angle = angle_between(x,x0,y,y0)
        velocity_vec.linear.x = 0.0
        velocity_vec.angular.z = 0.0

        if self.is_close_to_waypoint(x, x0, y, y0):
            velocity_vec.linear.x = 0.0
            velocity_vec.angular.z = 0.0
            self.state = STATE.REACHED
        else:
            if self.state == STATE.FOLLOW_WALL:
                velocity_vec = self.follow_wall(msg)
            elif self.state == STATE.MOVE_TO_GOAL:
                velocity_vec = self.move_to_goal(msg)

        self.publish_twist.publish(velocity_vec)

    def is_close_to_waypoint(self, x, x0, y, y0):
        distance = distance_between(x, x0, y, y0)
        return distance < self.goal_dist_thld

    def move_to_goal(self,msg):
        velocity_vec = Twist()
        x0 = robotX_tf
        y0 = robotY_tf

        x = self.waypoint.pose.position.x
        y = self.waypoint.pose.position.y

        distance = abs(0.5 * distance_between(x,x0,y,y0))
        angle = angle_between(x,x0,y,y0)

        if (abs(robot_yaw - angle) > 0.1):
            velocity_vec.linear.x = 0.0
            velocity_vec.angular.z = -0.3 if ((robot_yaw - angle) > 0) else 0.3
        else:
            velocity_vec.linear.x = 0.3
            velocity_vec.angular.z = 0.0

        if(msg.ranges[0] < 0.3):
            self.state = STATE.FOLLOW_WALL
            velocity_vec.linear.x = 0.0
            velocity_vec.angular.z = 0.0
            self.entry_point = [robotX_tf, robotY_tf]
            self.closest_point = [robotX_tf, robotY_tf]
        
        return velocity_vec

    def follow_wall(self, msg):
        velocity_vec = Twist()
        closest_angle = msg.ranges.index(min(msg.ranges[:]))
        turn_angle = (robot_yaw + math.radians(closest_angle + 90))

        if abs(robot_yaw - turn_angle) > math.pi:
            turn_angle += 2 * math.pi if ((robot_yaw - turn_angle) > 0) else -2 * math.pi
        if abs(robot_yaw - turn_angle) > 0.15:
            velocity_vec.linear.x = 0.1 if (msg.ranges[0] > 0.4) else 0.0
            velocity_vec.angular.z = -0.4 if ((robot_yaw - turn_angle) > 0) else 0.4
        else:
            quarter_length = len(msg.ranges)//4
            right_side_distance = msg.ranges[-quarter_length]
            left_side_distance = msg.ranges[quarter_length]
            velocity_vec.linear.x = 0.2

            if(right_side_distance<0.3):
                velocity_vec.angular.z = 0.5
            elif(left_side_distance<0.3):
                velocity_vec.angular.z = -0.5
            elif(right_side_distance>0.3):
                velocity_vec.angular.z = -0.5
            elif(left_side_distance>0.3):
                velocity_vec.angular.z = 0.5
            else:
                velocity_vec.linear.x = 0.4
                velocity_vec.angular.z = 0.0 

        self.estimated_distance += velocity_vec.linear.x
        x0 = robotX_tf
        y0 = robotY_tf

        x = self.waypoint.pose.position.x
        y = self.waypoint.pose.position.y

        current_distance_to_goal = distance_between(x0, x, y0, y)
        closest_distance = distance_between(self.closest_point[0], x, self.closest_point[1], y)

        if current_distance_to_goal < closest_distance:
            self.closest_point = [x0, y0]

        if abs(robotX_tf - self.entry_point[0]) < 0.1 and abs(robotY_tf - self.entry_point[1]) < 0.1 and self.estimated_distance > 5:
            self.go_to_closest_point = True
        
        if (abs(robotX_tf - self.closest_point[0]) < 0.1 and abs(robotY_tf - self.closest_point[1] < 0.1)) and self.go_to_closest_point:
            self.state = STATE.MOVE_TO_GOAL
            velocity_vec.linear.x = 0.0
            velocity_vec.angular.z = 0.0

        return velocity_vec


    def odom_callback(self, msg):
        global robotX # global keyword makes the variable accessable even outside the function!
        global robotY # global keyword makes the variable accessable even outside the function!
        global robotX_tf # global keyword makes the variable accessable even outside the function!
        global robotY_tf # global keyword makes the variable accessable even outside the function!
        global robot_yaw # global keyword makes the variable accessable even outside the function!
        
        robotX = msg.pose.pose.position.x
        robotY = msg.pose.pose.position.y
        
        to_frame_rel = "odom"
        from_frame_rel = "base_footprint"
        try:
            # grab the latest available transform from the odometry frame 
            # (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        _,_,robot_orient_z = euler_from_quaternion(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
        robotX_tf = t.transform.translation.x
        robotY_tf = t.transform.translation.y
        robot_yaw = robot_orient_z # # only need the z axis, degree of orientation, between pi and -pi
        #self.get_logger().info('X:'+str(robotX_tf),throttle_duration_sec=0.5) # once at a half of a second
        #self.get_logger().info('Y:'+str(robotY_tf),throttle_duration_sec=0.5) # once at a half of a second
        #self.get_logger().info('Yaw:'+str(robot_yaw),throttle_duration_sec=0.5) # once at a half of a second

        # TURTLEBOT: I HAVE NO CLUE, WHICH INFORMATION SHOULD I DEPEND FOR INTELLIGENT WAYPOINT FOLLOWING? HELP ME MY FELLOW ENGINEER! 
        # TURTLEBOT: I HAVE NO CLUE, WHICH INFORMATION SHOULD I DEPEND FOR INTELLIGENT WAYPOINT FOLLOWING? HELP ME MY FELLOW ENGINEER! 
        # Twist is a type of ROS Message that enables us to send velocity commands to the robot
        
        velocity_vec = Twist()
        self.flag = False

        #velocity_vec.linear.x = 0.0 # linear -> adjusting the velocity for driving forward or backwards
        #velocity_vec.angular.z = 0.0 # angular -> adjusting the velocity for turning the robot
        #self.publish_twist.publish(velocity_vec) # publish twist message through cmd_vel topic
        
        if (self.chatty_map):
            # you may set chatty_map to true if you want to see map on the terminal
            # map is only acquired for once and does not change since then.
            self.get_logger().info(str(self.occupancy_grid))
            self.get_logger().info("Length of the map array:" + str(len(self.occupancy_grid)))
            self.get_logger().info("Height:" + str(self.map_height) + " Width:"+ str(self.map_height))
            self.get_logger().info("Origin of the map (Cell 0,0):" + str(self.map_origin))
            self.get_logger().info("Resolution (Size of each grid cell):" + str(self.map_resolution))

            self.chatty_map = False # avoid repetitive printing.
        
        
def main(args=None):
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    rclpy.init(args=args)

    navigator_node = Navigator()
    navigator_node.send_request() # send request to map server
    get_response=False
    while rclpy.ok():
        rclpy.spin_once(navigator_node) 
        if (navigator_node.future.done() & (not get_response)):
            # if future job is accomplished (GetMap) and not accomplished before
            navigator_node.get_logger().info("map is acquired")
            try:
                response = navigator_node.future.result() # get map response
                get_response = True # raise the response flag
                navigator_node.occupancy_grid= response.map.data # get the occupancy grid array
                navigator_node.map_height= response.map.info.height # get the occupancy grid array
                navigator_node.map_width= response.map.info.width # get the occupancy grid array
                navigator_node.map_origin= response.map.info.origin # get the occupancy grid array
                navigator_node.map_resolution= response.map.info.resolution # get the occupancy grid array
                
            except Exception as e:
                navigator_node.get_logger().info(e) # raise an error if response could not be acquired.
                get_response = False # lower the flag


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()