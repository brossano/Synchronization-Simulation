import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy import qos
from sensor_msgs.msg import Image, Range
from geometry_msgs.msg import Twist 
import numpy as np
import matplotlib.pyplot as plt

import cv2
from cv_bridge import CvBridge
import apriltag
from simple_pid import PID

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.camera_subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.camera_callback,qos.qos_profile_sensor_data
            )
        self.camera_subscription  # prevent unused variable warning

        self.ultrasonic_subscription = self.create_subscription(
            Range,
            '/ultrasonic_sensor',
            self.ultrasonic_callback,qos.qos_profile_sensor_data
            )
        self.ultrasonic_subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_vel)

        self.dist_from_center = 0
        self.K_center = 0.004 # gain for control

        self.separation_dist = 0
        self.separation_dist_ref = 0.5
        self.K_separation = 0.5

        self.pid = PID(0.004, 0.0, 0.0, setpoint=0)

        self.dist_data = np.empty((0))
        

    def publish_vel(self):
        msg = Twist()
        # msg.linear.x = self.K_center*self.dist_from_center
        msg.linear.x = -1*self.pid(self.dist_from_center)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        # msg.angular.z = -1.0*self.K_separation*self.separation_dist

        self.publisher_.publish(msg)

    def camera_callback(self, msg):
        cv2.destroyAllWindows()
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(cv_image_gray)

        if len(results) == 0:
            print("No AprilTag Found")
            return

        tag = results[0]

        img_center_x = msg.width/2
        tag_center_x = tag.center[0]

        self.dist_from_center = img_center_x - tag_center_x
        
        print(f"Distance from Center: {self.dist_from_center}\n")
        # if self.dist_data.size == 100:
        #     self.dist_data = np.delete(self.dist_data, 0)
        # self.dist_data = np.append(self.dist_data, [self.dist_from_center])

        # plt.clf()
        # plt.plot(self.dist_data)
        # plt.plot(np.zeros(100), 'r')
        # plt.axis([0, 100, -300, 300])

        # plt.show(block=False)
        # plt.pause(0.001)


    def ultrasonic_callback(self, msg):
        self.separation_dist = msg.range - self.separation_dist_ref
        # print(f"Separation Distance: {self.separation_dist}\n")

            


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    rclpy.shutdown()


if __name__ == '__main__':
    main()