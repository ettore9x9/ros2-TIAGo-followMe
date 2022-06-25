# Author:
# - Addison Sears-Collins
# - Edited by: Pagano, Predieri, Sani
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
      
    # Create the subscriber. This subscriber will receive a ROS Image
    # from the 'kinect_color' topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'kinect_color', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning.

    # Create publisher
    # This node publishes the centroid pixel coordinates.
    self.publisher_centroid = self.create_publisher(PointStamped, 'centroid', 1)
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    self.out = cv2.VideoWriter(
        'output.avi',
        cv2.VideoWriter_fourcc(*'MJPG'),
        15.,
        (640,480))

    # Importing some pretrained face detection models.
    self.face_cascade_front = cv2.CascadeClassifier('./src/cv_basics/cv_basics/haarcascade_frontalface_alt2.xml')
    self.face_cascade_profile = cv2.CascadeClassifier('./src/cv_basics/cv_basics/haarcascade_profile.xml')

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Message to be published
    msg = PointStamped()
 
    # Convert ROS Image message to OpenCV image.
    current_frame = self.br.imgmsg_to_cv2(data)

    # Resizing for faster detection.
    frame = cv2.resize(current_frame, (640, 480)) # righe x colonne

    # Using a greyscale picture, also for faster detection.
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    
    # Detected faces are put into rectangles.
    faces = self.face_cascade_front.detectMultiScale(gray, 1.1, 4)

    # If no faces are detected, the training model becomes the face profiles. 
    if len(faces) == 0 :
      faces = self.face_cascade_profile.detectMultiScale(gray, 1.1, 4)


    for (x, y, w, h) in faces:

      cv2.rectangle(gray, (x, y), (x+w, y+h), (255, 0, 0), 2)   # Create the rectangle that will highlight the detected face.
      msg.header = data.header                                  # Populating the header field for timestamp comparison.
      msg.point.x = x+w/2                                       # x centroid coordinate
      msg.point.y = y+h/2                                       # y centroid coordinate
      self.get_logger().info('Centroid Pixel Coordinates = [' + str(msg.point.x) + ', ' + str(msg.point.y) + ']')
      # Publishing the message
      self.publisher_centroid.publish(msg)
      
    # Highlighting the centroid with a tiny rectangle in the centre of the face rectangle   
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