import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from interfaces.srv import SendPoint


class DepthFinder(Node):
  """
  Create an DepthFinder class, which is a subclass of the Node class.
  """

  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name.
    super().__init__('depth_finder')

    self.buffer = []
      
    # Create the subscriber. This subscriber will receive a ROS Image
    # from the kinect.
    self.sub_range = self.create_subscription(
      Image, 
      '/range_camera',
      self.range_cb, 
      10)

    # Create the subscriber. This subscriber will receive a PointStamped
    # point with the centroid coordinates in pixel.
    self.sub_centroid = self.create_subscription(
      PointStamped, 
      '/face_centre', 
      self.centroid_cb, 
      10)

    # Create a publisher
    # This node publishes the centroid depth and pixel coordinates as a Point message.
    # self.ctrl_publisher = self.create_publisher(Point, "control_msgs", 1)
      
    # Used to convert between ROS and OpenCV images.
    self.br = CvBridge()

    self.out = cv2.VideoWriter(
        'output.avi',
        cv2.VideoWriter_fourcc(*'MJPG'),
        15.,
        (640,480))

    self.cli = self.create_client(SendPoint, 'centroid')
    while not self.cli.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('service not available, waiting again...')

    self.ctrl_input = SendPoint.Request()
    self.client_futures = []

    self.spin()

  def range_cb(self, data):
    """
    Callback function.
    """

    # Stores all the frame images that are received in order to compare the timestamps at a later time.
    self.buffer.append(data)

    # Pop the last stored image if the buffer size gets too big.
    if len(self.buffer) > 100:
        self.buffer.pop(0)


  def centroid_cb(self, msg):
    """
    Callback function.
    """

    msg_nsec = msg.header.stamp.nanosec + 1000000000 * msg.header.stamp.sec
 
    for image in self.buffer:
        # Cycling into the frames buffer.
        image_nsec = image.header.stamp.nanosec + 1000000000 * image.header.stamp.sec

        # Here timestamps values are comapred in order to synchronize the received centroid with the corresponding frame.
        if abs(image_nsec - msg_nsec) < 100000000:

            # Convert ROS Image message to OpenCV image.
            current_frame = self.br.imgmsg_to_cv2(image, desired_encoding='passthrough')

            # Resizing for faster detection.
            frame = cv2.resize(current_frame, (640, 480))

            # Centroid coordinates.
            self.ctrl_input.centroid.x = msg.point.x
            self.ctrl_input.centroid.y = msg.point.y
            # Centroid depth
            self.ctrl_input.centroid.z = float(frame[int(msg.point.y), int(msg.point.x)])

            # # Publishing the centroid pixel coordinates and depth as a Point message.
            # self.ctrl_publisher.publish(self.ctrl_input)

            self.get_logger().info('Centroid Depth = ' + str(round(self.ctrl_input.centroid.z, 4)) + ' [m]')

            self.client_futures.append(self.cli.call_async(self.ctrl_input))

            #rclpy.spin_until_future_complete(self, self.future)

            return

        if msg_nsec > image_nsec:
            self.buffer.pop(0)

  def spin(self):
    while rclpy.ok():
      rclpy.spin_once(self)
      incomplete_futures = []
      for f in self.client_futures:
        if f.done():
          res = f.result()
          self.get_logger().info('Service SendPoint response:' + str(res.check))
        else:
          incomplete_futures.append(f)
      self.client_futures = incomplete_futures



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