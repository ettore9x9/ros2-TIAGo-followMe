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

from geometry_msgs.msg import Point
from interfaces.srv import SendPoint
from interfaces.srv import ProcessPoint
 
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

    # Create a subscriber
    # This node subscribes to messages of type 
    # Twist. 
    # These are the desired command velocities for centroid tracking
    # self.subscription = self.create_subscription(
    #                     Twist,
    #                     'des_cmd_vel',
    #                     self.update_desired_vel,
    #                     10)
    # self.subscription  # prevent unused variable warning
 
    # Create a subscriber
    # This node subscribes to messages of type 
    # sensor_msgs/LaserScan     
    self.scan_subscriber = self.create_subscription(
                           LaserScan,
                           '/scan',
                           self.scan_callback,
                           10)
                            
    # Create a publisher
    # This node publishes the linear and angular velocity of the robot.
    self.publisher_ = self.create_publisher(
                      Twist, 
                      '/cmd_vel', 
                      10)

    # Desired linear and angular velocities for centroid tracking
    self.vel_x = 0.0
    self.vel_z = 0.0

    # Increment the frequency of the self.collision_avoidance execution
    # self.timer = self.create_timer(0.1, self.collision_avoidance)

    # Initialize the LaserScan sensor readings to some large value
    # Values are in meters.
    self.max_distance = 5.0
    self.front_dist = self.max_distance

    self.srv = self.create_service(SendPoint, 'centroid', self.receive_point_callback)
    self.pid_request = ProcessPoint.Request()

    self.cli = self.create_client(ProcessPoint, 'des_vel')
    while not self.cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')

    self.data_received = 0

    ############# OBSTALCE AVOIDANCE PARAMETERS #######################     
 
    # Distance threshold.
    # We want to try to keep within this distance from the wall.
    self.dist_thresh_wf = 1.1 # meters

    self.collision_avoidance()

  # def update_desired_vel(self, msg):
  #   """
  #   This method updates both linear and angular velocities for centroid tracking
  #   """
  #   self.vel_x = msg.linear.x
  #   self.vel_z = msg.angular.z

  def receive_point_callback(self, req, res):
    self.get_logger().info('Received centroid')
    self.pid_request.centroid = req.centroid
    self.data_received = 1
    res.check = True
    return res

  def scan_callback(self, msg):
    """
    This method gets called every time a LaserScan message is 
    received.
    """
    self.get_logger().info('Received laserscan')
    self.front_dist = min(msg.ranges[260:400])
   
  def collision_avoidance(self):
    """
    This method causes the robot to follow the boundary of a wall.
    """
    # Create a geometry_msgs/Twist message
    msg = Twist()

    while rclpy.ok():
      rclpy.spin_once(self)

      if self.data_received == 1:

        self.future = self.cli.call_async(self.pid_request)

        self.get_logger().info('called pid controller')

        rclpy.spin_until_future_complete(self, self.future)

        self.get_logger().info('Received pid controller response')

        res = self.future.result()
        self.vel_x = res.desired_velocities.linear.x
        self.vel_z = res.desired_velocities.angular.z

      else:
        self.vel_x = 0.8 * self.vel_x
        self.vel_z = 0.8 * self.vel_z

      # Logic for following the wall
      # >d means no wall detected by that laser beam
      # <d means an wall was detected by that laser beam
      d = self.dist_thresh_wf

      # Only data front laser scan data are evaluated so as to avoid losing sight of the centroid
      if self.front_dist > d:
        self.wall_following_state = "1: No obstacles detected"
        msg.linear.x = self.vel_x
        msg.angular.z = self.vel_z

      else:
        self.wall_following_state = "2: Obstacles detected at " + str(round(self.front_dist, 3)) + " meters in front of the robot."
        msg.linear.x = 0.0
        msg.angular.z = self.vel_z
  
      # Send velocity command to the robot
      self.publisher_.publish(msg)


      self.data_received = 0
    
      # Print robot state information in terminal
      self.get_logger().info('State: "%s"' % self.wall_following_state)
      # self.get_logger().info('vel x: "%f" vel z "%f"' % self.vel_x % self.vel_z)

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
