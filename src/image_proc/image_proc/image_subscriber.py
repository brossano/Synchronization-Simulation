import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy import qos
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist 
import numpy as np
import cv2
from cv_bridge import CvBridge
import apriltag

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.listener_callback,qos.qos_profile_sensor_data
            )
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_vel)

        self.dist_from_center = 0
        self.K = 0.003 # gain for control
        

    def publish_vel(self):
        msg = Twist()
        msg.linear.x = self.K*self.dist_from_center
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.publisher_.publish(msg)

    def listener_callback(self, msg):
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

        print(f"Distance from Center: {img_center_x - tag_center_x}")

        # img = np.asarray(cv_image)
        # num_green = np.sum(img(img[:,:,0] > 200))
            


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()