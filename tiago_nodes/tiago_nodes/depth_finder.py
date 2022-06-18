import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point

class DepthFinder(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """

  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('depth_finder')

    self.buffer = []
    self.depth = Point()
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.sub_range = self.create_subscription(
      Image, 
      '/TIAGo_Iron/kinect_range', 
      self.range_cb, 
      10)

    self.sub_centroid = self.create_subscription(
      PointStamped, 
      '/TIAGo_Iron/centroid', 
      self.centroid_cb, 
      10)
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    # self.hog = cv2.HOGDescriptor()
    # self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    self.out = cv2.VideoWriter(
        'output.avi',
        cv2.VideoWriter_fourcc(*'MJPG'),
        15.,
        (640,480))

  def range_cb(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving range frame ' + str(len(self.buffer)))

    self.buffer.append(data)

    if len(self.buffer) > 100:
        self.buffer.pop(0)


  def centroid_cb(self, msg):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving centroid')

    msg_nsec = msg.header.stamp.nanosec + 1000000000 * msg.header.stamp.sec
 
    for image in self.buffer:

        image_nsec = image.header.stamp.nanosec + 1000000000 * image.header.stamp.sec

        if abs(image_nsec - msg_nsec) < 100000000:

            # Convert ROS Image message to OpenCV image
            current_frame = self.br.imgmsg_to_cv2(image, desired_encoding='passthrough')

            # resizing for faster detection
            frame = cv2.resize(current_frame, (640, 480))

            # centroid coordinates
            self.depth.x = msg.point.y
            self.depth.y = msg.point.x
            # centroid depth
            self.depth.z = float(frame[int(msg.point.y), int(msg.point.x)])

            self.get_logger().info('depth = ' + str(frame[int(msg.point.y),int(msg.point.x)]))

            return

        if msg_nsec > image_nsec:
            self.buffer.pop(0)


def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  depth_finder = DepthFinder()
  
  # Spin the node so the callback function is called.
  rclpy.spin(depth_finder)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  depth_finder.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
    main()