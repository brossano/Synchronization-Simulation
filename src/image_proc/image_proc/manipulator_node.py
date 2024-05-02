import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

from mbot_xl320_library import *
import time

# Define your settings here
# CONNECTION_DEVICE = "UART"    # change to "UART" if you are using UART connection
CONNECTION_DEVICE = "USB"   # change to "USB" if you are using USB2AX connection
# PORT_NAME = "/dev/ttyTHS1"    # UART has fixed port name ttyTHS1, 
PORT_NAME = "/dev/ttyACM3"  # USB port names are dynamic you need to check what it is

#SERVO 1 LIMIT: 0 - 1023 (0 - 300 deg)
#SERVO 2 LIMIT: 205 - 922 (60 - 270 deg)
#SERVO 3 LIMIT: 0 - 818 (0 - 240 deg)
#SERVO 4 LIMIT: 0 - 1023 (0 - 300 deg)
#SERVO 5 LIMIT: 205 - 818 (60 - 240 deg)

class ManipulatorROS(Node):

    def __init__(self):
        super().__init__('manipulator_ros')
        self.subscription = self.create_subscription(
            Int32,
            'manipulator_state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # ARM INITIALIZATION
        if CONNECTION_DEVICE == "UART":
            initialize_GPIO()
            portHandler, packetHandler = initialize_gpio_handlers(PORT_NAME)
        elif CONNECTION_DEVICE == "USB":
            portHandler, packetHandler = initialize_handlers(PORT_NAME)
        else:
            print("Invalid connnection device!")

        # defines the servo's ID
        self.servo1_ID = 1
        servo2_ID = 2
        self.servo3_ID = 3
        self.servo4_ID = 4
        self.servo5_ID = 5
        
        # define speed
        speed = 150

        open_port(portHandler)
        set_baudrate(portHandler, 1000000)

        self.servo1 = Servo(self.servo1_ID, portHandler, packetHandler)
        self.servo1.change_led_color(LED_RED)
        self.servo1.disable_torque()
        self.servo1.set_control_mode("joint")  # torque must be off when you change mode
        self.servo1.set_joint_speed(speed)
        self.servo1.enable_torque()

        self.servo2 = Servo(servo2_ID, portHandler, packetHandler)
        self.servo2.change_led_color(LED_YELLOW)
        self.servo2.disable_torque()
        self.servo2.set_control_mode("joint")  # torque must be off when you change mode
        self.servo2.set_joint_speed(speed)
        self.servo2.enable_torque()
        
        self.servo3 = Servo(self.servo3_ID, portHandler, packetHandler)
        self.servo3.change_led_color(LED_CYAN)
        self.servo3.disable_torque()
        self.servo3.set_control_mode("joint")  # torque must be off when you change mode
        self.servo3.set_joint_speed(speed - 50)
        self.servo3.enable_torque()

        self.servo4 = Servo(self.servo4_ID, portHandler, packetHandler)
        self.servo4.change_led_color(LED_PURPLE)
        self.servo4.disable_torque()
        self.servo4.set_control_mode("joint")  # torque must be off when you change mode
        self.servo4.set_joint_speed(speed)
        self.servo4.enable_torque()
        
        self.servo5 = Servo(self.servo5_ID, portHandler, packetHandler)
        self.servo5.change_led_color(LED_GREEN)
        self.servo5.disable_torque()
        self.servo5.set_control_mode("joint")  # torque must be off when you change mode
        self.servo5.set_joint_speed(speed)
        self.servo5.enable_torque()
        
    #     self.servo1.disable_torque()
    #     self.servo2.disable_torque()
    #     self.servo3.disable_torque()
    #     self.servo4.disable_torque()
    #     self.servo5.disable_torque()

        #idle
        self.idlePos()
        time.sleep(2)

        
    def degToCmd(self,deg):
        return int(3.41329*deg)

    def cmdToDeg(self,cmd):
        return cmd/3.41329

    def openGripper(self,servo):
        servo.set_position(self.degToCmd(120))
        
    def closeGripper(self,servo):
        servo.set_position(self.degToCmd(165))

    def idlePos(self):
        self.servo3.set_position(self.degToCmd(150))
        self.servo4.set_position(self.degToCmd(180))
        time.sleep(0.25)
        self.servo1.set_position(self.degToCmd(150))
        self.servo2.set_position(self.degToCmd(150))
        self.servo5.set_position(self.degToCmd(120))
        
    def grabPos(self):
        self.servo3.set_position(self.degToCmd(80))
        time.sleep(0.5)
        self.servo1.set_position(self.degToCmd(60))
        self.servo2.set_position(self.degToCmd(150))
        self.servo4.set_position(self.degToCmd(185))
        time.sleep(1)
        self.servo3.set_position(self.degToCmd(120))
        time.sleep(0.75)
        self.closeGripper(self.servo5)
        
    def idleHold(self):
        self.servo3.set_position(self.degToCmd(80))
        time.sleep(1)
        self.servo1.set_position(self.degToCmd(150))
        self.servo3.set_position(self.degToCmd(150))
        self.servo4.set_position(self.degToCmd(180))
        time.sleep(0.25)
        self.servo2.set_position(self.degToCmd(150))
        
    def releasePos(self):
        self.servo2.set_position(self.degToCmd(150))
        self.servo3.set_position(self.degToCmd(60))
        self.servo4.set_position(self.degToCmd(150))
        time.sleep(1)
        self.servo1.set_position(self.degToCmd(150))
        time.sleep(1)
        self.servo3.set_position(self.degToCmd(0))
        self.servo4.set_position(self.degToCmd(120))
        time.sleep(1)
        self.openGripper(self.servo5)

    def listener_callback(self, msg):
        if msg.data == 0:
            #grab and hold
            self.grabPos()
            time.sleep(1)
            self.idleHold()
            time.sleep(2)
        elif msg.data == 1:
            #release and idle
            self.releasePos()
            time.sleep(1)
            self.idlePos()
            time.sleep(2)




def main(args=None):
    rclpy.init(args=args)

    manipulator_ros = ManipulatorROS()

    rclpy.spin(manipulator_ros)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    manipulator_ros.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()