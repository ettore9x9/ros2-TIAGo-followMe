# Author: Addison Sears-Collins
# Date: March 19, 2021
# ROS Version: ROS 2 Foxy Fitzroy
 
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
                           qos_profile=qos_profile_sensor_data)
                            
    # Create a publisher
    # This node publishes the desired linear and angular velocity of the robot (in the
    # robot chassis coordinate frame) to the /demo/cmd_vel topic. Using the diff_drive
    # plugin enables the robot model to read this /demo/cmd_vel topic and execute
    # the motion accordingly.
    self.publisher_ = self.create_publisher(
                      Twist, 
                      '/cmd_vel', 
                      10)

    self.vel_x = 0
    self.vel_z = 0
 
    # Initialize the LaserScan sensor readings to some large value
    # Values are in meters.
    self.max_distance = 3.5
    self.crashed_min = 0.30
    self.crashed = False

    self.left_dist = self.max_distance # Left
    self.leftfront_dist = self.max_distance # Left-front

    self.front_dist = self.max_distance
    self.front_dist_1 = self.max_distance # Front
    self.front_dist_2 = self.max_distance # Front

    self.behind_dist = self.max_distance # Behind

    self.rightfront_dist = self.max_distance # Right-front
    self.right_dist = self.max_distance # Right
 
    ################### ROBOT CONTROL PARAMETERS ##################
    # Maximum forward speed of the robot in meters per second
    # Any faster than this and the robot risks falling over.
    self.forward_speed = 0.25
 
    # Current position and orientation of the robot in the global 
    # reference frame
    self.current_x = 0.0
    self.current_y = 0.0
    self.current_yaw = 0.0
 
    ############# WALL FOLLOWING PARAMETERS #######################     
    # Finite states for the wall following mode
    #  "turn left": Robot turns towards the left
    #  "search for wall": Robot tries to locate the wall        
    #  "follow wall": Robot moves parallel to the wall
    #self.wall_following_state = "turn left"
         
    # Set turning speeds (to the left) in rad/s 
    # These values were determined by trial and error.
    self.turning_speed_wf_fast = 0.5  # Fast turn
    #self.turning_speed_wf_slow = 0.2 # Slow turn
 
    # Wall following distance threshold.
    # We want to try to keep within this distance from the wall.
    self.dist_thresh_wf = 0.8 # in meters  
 
    # We don't want to get too close to the wall though.
    self.dist_too_close_to_wall = 0.5 # in meters

  # def logger_callback(self):
  #   self.get_logger().info('Publishing: "%lf"' % front_distance)
 
  def update_desired_vel(self, msg):
    self.vel_x = msg.linear.x
    self.vel_z = msg.angular.z
 
  def scan_callback(self, msg):
    """
    This method gets called every time a LaserScan message is 
    received on the '/demo/laser/out' topic 
    """
    
    self.left_dist = min(msg.ranges[60:100])
    self.leftfront_dist = min(msg.ranges[30:60])
    
    self.rightfront_dist = min(msg.ranges[300:330])
    self.right_dist = min(msg.ranges[260:300])

    self.front_dist_1 = min(msg.ranges[0:30])
    self.front_dist_2 = min(msg.ranges[330:359])
    self.front_dist = min(self.front_dist_1, self.front_dist_2)

    self.behind_dist = min(msg.ranges[180:270])

    if self.front_dist_1 < self.crashed_min or self.rightfront_dist < self.crashed_min or self.right_dist < self.crashed_min :
      if self.behind_dist > 1.0 :
        self.crashed = True
      else :
        self.crashed = False  
   
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
    e = self.dist_too_close_to_wall

    if self.front_dist > e and self.rightfront_dist > e and self.right_dist > e:
      if self.front_dist > d and self.leftfront_dist > d and self.rightfront_dist > d:
        self.wall_following_state = "1: No obstacles detected"
        msg.linear.x = self.vel_x
        msg.angular.z = self.vel_z
    
      elif self.front_dist < d and self.leftfront_dist > d  and self.rightfront_dist > d:
        self.wall_following_state = "2: turn"
        msg.linear.x = 0.0
        msg.angular.z = self.vel_z
    
      elif self.front_dist > d and self.leftfront_dist > d and self.rightfront_dist < d:
        self.wall_following_state = "3: turn left"
        msg.linear.x = self.vel_x
        msg.angular.z = self.turning_speed_wf_fast      
    
      elif self.front_dist > d and self.leftfront_dist < d and self.rightfront_dist > d:
        self.wall_following_state = "4: turn right"
        msg.linear.x = self.vel_x
        msg.angular.z = -self.turning_speed_wf_fast 
    
      elif self.front_dist < d and self.leftfront_dist > d and self.rightfront_dist < d:
        self.wall_following_state = "5: turn left"
        msg.linear.x = 0.0
        msg.angular.z = self.turning_speed_wf_fast
    
      elif self.front_dist < d and self.leftfront_dist < d and self.rightfront_dist > d:
        self.wall_following_state = "6: turn right"
        msg.linear.x = 0.0
        msg.angular.z = -self.turning_speed_wf_fast
    
      elif self.front_dist < d and self.leftfront_dist < d and self.rightfront_dist < d:
        self.wall_following_state = "7: obstacles detected everywhere"
        msg.linear.x = 0.0
        msg.angular.z = 0.0
                
      elif self.front_dist > d and self.leftfront_dist < d and self.rightfront_dist < d:
        self.wall_following_state = "8: turn right"
        msg.linear.x = self.vel_x
        msg.angular.z = 0.0
      
      else :
        self.wall_following_state = "9: move backward"
        msg.linear.x = -1.0
        msg.angular.z = 0.0
 
    # Send velocity command to the robot
    self.publisher_.publish(msg)  
    
    # Send robot state information in terminal
    self.get_logger().info('Robot State: "%s"' % self.wall_following_state)
    
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
