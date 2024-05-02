import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy import qos
from sensor_msgs.msg import Image, Range
from geometry_msgs.msg import Twist 
from std_msgs.msg import Bool, Int32, Float64
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
        self.bridge = CvBridge()

        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,qos.qos_profile_sensor_data
            )
        self.camera_subscription  # prevent unused variable warning

        self.ultrasonic_subscription = self.create_subscription(
            Float64,
            '/ultrasonic',
            self.ultrasonic_callback,qos.qos_profile_sensor_data
            )
        self.ultrasonic_subscription  # prevent unused variable warning

        self.sync_subscription = self.create_subscription(
            Bool,
            '/sync',
            self.sync_callback,qos.qos_profile_sensor_data)
        self.sync_subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.manipulator_publisher_ = self.create_publisher(Int32, '/manipulator_state', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_vel)

        self.dist_from_center = 0
        self.K_center = 0.004 # gain for control

        self.separation_dist_ref = .2
        self.ult_range = 0
        self.pid_cam = PID(0.0008, 0.0000, 0.0005, setpoint=0)

        self.pid_ult = PID(.1, 0.000, 0.000, setpoint=self.separation_dist_ref)

        self.start = True

        self.ult_counter = 0

        # for testing individual parts
        # self.sync = True

        # for whole sim
        self.sync = False

        self.in_frame = False
        self.dist_data = np.empty((0))

        self.sync_counter = 0
        # plt.ion()
        # self.fig = plt.figure()
        # self.ax = plt.subplot(1,1,1)
        # self.ax.set_xlim(0,200)
        # # self.ax.set_ylim(-.4,.4)
        # self.ax.set_ylim(-300,300)
        # self.ax.plot(np.zeros(200), 'r')     
        # self.fig.show()
        # self.fig.canvas.flush_events()
        # self.x = range(1,201)

         

    def sync_callback(self, msg):
        self.sync = msg.data

    def publish_vel(self):
        if not self.sync:
            return
        
        msg = Twist()
        if self.start:
            if self.in_frame:
                msg.linear.x = 0.05
                self.publisher_.publish(msg)
            self.start = self.dist_from_center <= 0.0
            return
        # msg.linear.x = self.K_center*self.dist_from_center
        input = -1*self.pid_cam(self.dist_from_center)
        if input <= 0:
            input = 0.02
        # print(input)
        msg.linear.x = input
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        msg.angular.z = 0.0
        msg.angular.z = self.pid_ult(self.ult_range)

        self.publisher_.publish(msg)
        self.sync_counter += 1

        if self.sync_counter == 50:
            manipulator_state = Int32()
            manipulator_state.data = 1
            self.manipulator_publisher_.publish(manipulator_state)


    def camera_callback(self, msg):
        if not self.sync:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("camera", cv_image_gray)   
        # cv2.waitKey(1)

        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(cv_image_gray)

        if len(results) == 0:
            # print("No AprilTag Found")
            return

        self.in_frame = True
        tag = results[0]

        img_center_x = msg.width/2
        tag_center_x = tag.center[0]

        self.dist_from_center = img_center_x - tag_center_x
        
        # print(f"Distance from Center: {self.dist_from_center - 80}\n")

        # if self.dist_data.size == 200:
        #     self.dist_data = np.delete(self.dist_data, 0)

  
        # self.dist_data = np.append(self.dist_data, [self.dist_from_center])
        # self.ax.clear()
        # self.ax.set_xlim(0,200)
        # self.ax.set_ylim(-300,300)
        # self.ax.plot(self.dist_data, 'k-')
        # self.ax.plot(np.zeros(200), 'r')
        # self.ax.set_xlabel('Control Iteration')
        # self.ax.set_ylabel('Pixels')
        # self.fig.canvas.flush_events()


    def ultrasonic_callback(self, msg):
        if not self.sync:
            return
   
        # if msg.data > 0.4:
        #     self.ult_range = self.separation_dist_ref
        #     return
            
        self.ult_counter += 1
        if self.ult_counter < 50:
            self.ult_range = self.separation_dist_ref

        if not msg.data:
            self.ult_range = self.separation_dist_ref

        self.ult_range = msg.data

        separation_dist = msg.data - self.separation_dist_ref

        # if self.dist_data.size == 400:
        #     self.dist_data = np.delete(self.dist_data, 0)

        # self.dist_data = np.append(self.dist_data, [separation_dist])
        # self.ax.clear()
        # self.ax.set_xlim(0,400)
        # self.ax.set_ylim(-.4,.4)
        # self.ax.plot(self.dist_data, 'k-')
        # self.ax.plot(self.separation_dist_ref*np.zeros(400), 'r')
        # self.ax.set_xlabel('Control Iteration')
        # self.ax.set_ylabel('Distance from Setpoint')
        # self.fig.canvas.flush_events()

        # print(f"Separation Distance: {msg.range}")

            


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    rclpy.shutdown()


if __name__ == '__main__':
    main()