# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from geometry_msgs.msg import PointStamped
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'kinect_color', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    self.publisher_centroid = self.create_publisher(PointStamped, 'centroid', 1)
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    # self.hog = cv2.HOGDescriptor()
    # self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    self.out = cv2.VideoWriter(
        'output.avi',
        cv2.VideoWriter_fourcc(*'MJPG'),
        15.,
        (640,480))

    self.face_cascade = cv2.CascadeClassifier('./src/cv_basics/cv_basics/haarcascade_frontalface_alt2.xml')

  def listener_callback(self, data):
    """
    Callback function.
    """
    msg = PointStamped()
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)

    # resizing for faster detection
    frame = cv2.resize(current_frame, (640, 480))

    # using a greyscale picture, also for faster detection
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    
    # detect people in the image
    # returns the bounding boxes for the detected objects
    # boxes, weights = self.hog.detectMultiScale(gray, winStride=(8,8))

    # boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

    # for (xA, yA, xB, yB) in boxes:
    #     # display the detected boxes in the colour picture
    #     cv2.rectangle(gray, (xA, yA), (xB, yB),(0, 255, 0), 2)


    faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)

    for (x, y, w, h) in faces:
      cv2.rectangle(gray, (x, y), (x+w, y+h), (255, 0, 0), 2)
      msg.header = data.header
      msg.point.x = x+w/2
      msg.point.y = y+h/2
      self.publisher_centroid.publish(msg)

    cv2.rectangle(gray, (int(msg.point.x),int(msg.point.y)), (int(msg.point.x)+1, int(msg.point.y)+1), (255, 0, 0), 2)
    # Write the output video 
    self.out.write(gray.astype('uint8'))

    # Display image
    cv2.imshow("camera", gray)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()