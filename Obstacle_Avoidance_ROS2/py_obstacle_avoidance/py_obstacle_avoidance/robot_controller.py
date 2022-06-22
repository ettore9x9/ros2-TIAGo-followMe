# Author: Addison Sears-Collins
# Edited by: Pagano, Predieri, Sani
# Date: June 22, 2022
# ROS Version: ROS 2 Foxy Fitzroy Galactic
 
############## IMPORT LIBRARIES #################
# Python math library
import math 
 
# ROS client library for Python
import rclpy 
 
# Enables pauses in the execution of code
from time import sleep 
 
# Used to create nodes
from rclpy.node import Node
 
# Enables the use of the string message type
from std_msgs.msg import String 
 
# Twist is linear and angular velocity
from geometry_msgs.msg import Twist     
                     
# Handles LaserScan messages to sense distance to obstacles (i.e. walls)        
from sensor_msgs.msg import LaserScan    
 
# Handle Pose messages
from geometry_msgs.msg import Pose 
 
# Handle float64 arrays
from std_msgs.msg import Float64MultiArray
                     
# Handles quality of service for LaserScan data
from rclpy.qos import qos_profile_sensor_data 
 
# Scientific computing library
import numpy as np 
 
class Controller(Node):
  """
  Create a Controller class, which is a subclass of the Node 
  class for ROS2.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    ##################### ROS SETUP ####################################################
    # Initiate the Node class's constructor and give it a name
    super().__init__('Controller')
 

    self.subscription = self.create_subscription(
                        Twist,
                        'des_cmd_vel',
                        self.update_desired_vel,
                        10)
    self.subscription  # prevent unused variable warning
 
    # Create a subscriber
    # This node subscribes to messages of type 
    # sensor_msgs/LaserScan     
    self.scan_subscriber = self.create_subscription(
                           LaserScan,
                           '/scan',
                           self.scan_callback,
                           10)
                            
    # Create a publisher
    # This node publishes the desired linear and angular velocity of the robot (in the
    # robot chassis coordinate frame) to the /demo/cmd_vel topic. Using the diff_drive
    # plugin enables the robot model to read this /demo/cmd_vel topic and execute
    # the motion accordingly.
    self.publisher_ = self.create_publisher(
                      Twist, 
                      '/cmd_vel', 
                      10)

    self.vel_x = 0.0
    self.vel_z = 0.0
    self.last_turn = ''

    self.timer = self.create_timer(0.01, self.follow_wall)
    # Initialize the LaserScan sensor readings to some large value
    # Values are in meters.
    self.max_distance = 5.0
    self.crashed = False

    # self.left_dist = self.max_distance # Left
    # self.leftfront_dist = self.max_distance # Left-front
    self.front_dist = self.max_distance
    # self.behind_dist = self.max_distance # Behind
    # self.rightfront_dist = self.max_distance # Right-front
    # self.right_dist = self.max_distance # Right
 
    ################### ROBOT CONTROL PARAMETERS ##################

    # Maximum forward speed of the robot in meters per second
    # Any faster than this and the robot risks falling over.

 
    ############# OBSTALCE AVOIDANCE PARAMETERS #######################     
 
    # Distance threshold.
    # We want to try to keep within this distance from the wall.
    self.dist_thresh_wf = 1.1 # meters  

    # We don't want to get too close to the wall though.
 
  def update_desired_vel(self, msg):
    self.vel_x = msg.linear.x
    self.vel_z = msg.angular.z
 
  def scan_callback(self, msg):
    """
    This method gets called every time a LaserScan message is 
    received.
    """

    #self.leftfront_dist = min(msg.ranges[0:35])
    #self.rightfront_dist = min(msg.ranges[625:660])
    self.front_dist = min(msg.ranges[310:350])
   
    # Logic for following the wall
    # >d means no wall detected by that laser beam
    # <d means an wall was detected by that laser beam

    self.follow_wall()
   
  def follow_wall(self):
    """
    This method causes the robot to follow the boundary of a wall.
    """
    # Create a geometry_msgs/Twist message
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0        
 
    d = self.dist_thresh_wf

    if self.front_dist > d:
      self.wall_following_state = "1: No obstacles detected"
      msg.linear.x = self.vel_x
      msg.angular.z = self.vel_z
      print("self.vel_x = " + str(self.vel_x))
      print("self.vel_z = " + str(self.vel_z))

    #   if self.vel_x < 0.000001 and self.vel_z < 0.000001 and self.last_turn == 'left':
    #     # search for target
    #     msg.linear.x = self.vel_x
    #     msg.angular.z = -self.turning_speed_wf_fast
    #   if self.vel_x < 0.000001 and self.vel_z < 0.000001 and self.last_turn == 'right':
    #     # search for target
    #     msg.linear.x = self.vel_x
    #     msg.angular.z = self.turning_speed_wf_fast

    # if self.front_dist > d and (self.leftfront_dist < d or self.rightfront_dist < d):
    #   self.wall_following_state = "2: Obstacle sidewards"
    #   msg.linear.x = self.forward_speed
    #   if self.rightfront_dist < d:
    #     msg.angular.z = self.turning_speed_wf_fast
    #   elif self.leftfront_dist < d:
    #     msg.angular.z = -self.turning_speed_wf_fast
  
    # elif self.front_dist > d and self.leftfront_dist > d and self.rightfront_dist < d:
    #   self.wall_following_state = "3: turn left"
    #   msg.linear.x = self.vel_x
    #   msg.angular.z = self.turning_speed_wf_fast 
    #   self.last_turn = 'left'     
  
    # elif self.front_dist > d and self.leftfront_dist < d and self.rightfront_dist > d:
    #   self.wall_following_state = "4: turn right"
    #   msg.linear.x = self.vel_x
    #   msg.angular.z = -self.turning_speed_wf_fast
    #   self.last_turn = 'right'
  
    # elif self.front_dist < d and self.leftfront_dist > d and self.rightfront_dist < d:
    #   self.wall_following_state = "5: turn left"
    #   msg.linear.x = 0.0
    #   msg.angular.z = self.turning_speed_wf_fast
    #   self.last_turn = 'left'
  
    # elif self.front_dist < d and self.leftfront_dist < d and self.rightfront_dist > d:
    #   self.wall_following_state = "6: turn right"
    #   msg.linear.x = 0.0
    #   msg.angular.z = -self.turning_speed_wf_fast
    #   self.last_turn = 'right'
  
    # elif self.front_dist < d and self.leftfront_dist < d and self.rightfront_dist < d:
    #   self.wall_following_state = "7: obstacles detected everywhere"
    #   msg.linear.x = 0.0
    #   msg.angular.z = 0.0

    else:
      self.wall_following_state = "2: Obstacles detected at " + str(round(self.front_dist, 3)) + " meters in front of the robot."
      msg.linear.x = 0.0
      msg.angular.z = self.vel_z
    
    # Send velocity command to the robot

    self.publisher_.publish(msg)  
    print(str(self.front_dist))
    # Send robot state information in terminal
    self.get_logger().info('State: "%s"' % self.wall_following_state)
    
def main(args=None):
 
    # Initialize rclpy library
    rclpy.init(args=args)
     
    # Create the node
    controller = Controller()
 
    # Spin the node so the callback function is called
    # Pull messages from any topics this node is subscribed to
    # Publish any pending messages to the topics
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
     
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
