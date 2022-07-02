import rclpy # Python library for ROS 2
from simple_pid import PID
import time
import numpy as np
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Point, Twist
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from interfaces.srv import ProcessPoint

class Controller(Node):

  def __init__(self):
    """
    Class constructor to set up the node.
    """

    # Initiate the Node class's constructor and give it a name.
    super().__init__('pid_controller')

    self.y = 0.0
    self.depth = 5.0
    self.x = 0.0

    # Create publisher.
    # This node publishes the desired tilt head joint trajectory in order to keep tracking the centroid .
    self.cmd_vel_joint_publisher = self.create_publisher(JointTrajectory, "/command_pose_head", 1)

    # Increase Hertz execution.
    self.timer = self.create_timer(0.1, self.head_angle)

    # Implementing the two PD controllers for the centroid tracking.
    # Depth PD controller.
    # The robot should keep a 2m distance from target (centroid).
    self.pid_distance = PID(-0.2, 0, -0.4, setpoint=2)

    # Orientation PD controller.
    # The x centroid coordinate should be the half the the image width (i.e. pixel number 320).
    self.pid_orientation = PID(0.0008, 0, 0.0016, setpoint=320)

    # Message for publishing the desired command velocity.
    self.msg = Twist()

    # Message for publishing the desired head joint trajectory.
    self.msg_joint = JointTrajectory()

    # Populating the trajectory messages fields.
    self.msg_joint.joint_names = ["head_2_joint"]
    self.traj_points = JointTrajectoryPoint()
    self.traj_points.time_from_start.sec = 0
    self.traj_points.velocities = [0.1]
    self.traj_points.positions = [0.1]

    # Create a server of the des_vel service, to process the client (robot_controller.py) request
    # composed of the centroid position and depth.
    self.srv = self.create_service(ProcessPoint, 'des_vel', self.control_cb)

    
  def control_cb(self, req, res):
    """
    Callback function.
    """
    # Reading the centroid coordinates and depth.
    self.x = req.centroid.x
    self.y = req.centroid.y
    self.depth = req.centroid.z

    # Sending the target tracking required velocities to the client node (robot_controller.py).  
    res.desired_velocities.linear.x = float(self.pid_distance(self.depth))
    res.desired_velocities.angular.z = float(self.pid_orientation(self.x))
    return res

  def head_angle(self):
    """
    This method makes some simple joint head movement.  
    """

    # The y coordinates is on top of the image and TIAGo is pretty close to the target
    if self.y <= 160 and self.depth < 3.2 and self.depth > 0.5:

      self.traj_points.positions = [0.3]  # Raises TIAGo's head

    # The y coordinates is in the middle the image and TIAGo is pretty far from the target
    elif self.y > 180 and self.y <= 340 and self.depth > 3.4:

      self.traj_points.positions = [0.1]  # Alligns TIAGo's head

    self.msg_joint.points = [self.traj_points]
    self.cmd_vel_joint_publisher.publish(self.msg_joint)

    self.get_logger().info('Control Errors [' + str(self.x - 320) +  ' [px], ' + str(round(self.depth - 2, 4)) + ' [m] ]')

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  controller = Controller()
  
  # Spin the node so the callback function is called.
  rclpy.spin(controller)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  controller.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
    main()