from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library




class adapter():

	def __init__(self):

		self.br = CvBridge()


	def frame_adapter(ros_image):

		cv2_image = self.br.imgmsg_to_cv2(ros_image)
		rz_cv2_image = cv2.resize(cv2_image, (640, 480))
		return rz_cv2_image
	
	