#! /usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from gst_cam import camera
import cv2 
	
class ImagePublisher(Node):
	def __init__(self):
		super().__init__('image_publisher')

		topic_name= '/camera/image_raw'

		self.publisher_ = self.create_publisher(Image, topic_name , 10)
		self.timer = self.create_timer(0.1, self.timer_callback)

		self.cap = cv2.VideoCapture(0)
		self.br = CvBridge()

		# self.subscription = self.create_subscription(Image, topic_name, self.img_callback, 10)
		# self.subscription 
		self.br = CvBridge()


	def timer_callback(self):
		ret, frame = self.cap.read()   
		# frame = cv2.flip(frame, 1)  
		
		if not ret:
			return
			
		self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
	

	# def img_callback(self, data):
	#     self.get_logger().info('Receiving video frame')
	#     current_frame = self.br.imgmsg_to_cv2(data)
	#     cv2.imshow("camera", current_frame)   
	#     cv2.waitKey(1)


def main(args=None):
	rclpy.init(args=args)
	image_publisher = ImagePublisher()
	rclpy.spin(image_publisher)
	image_publisher.destroy_node()
	rclpy.shutdown()

  
if __name__ == '__main__':
  main()
