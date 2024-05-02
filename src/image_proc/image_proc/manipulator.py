from mbot_xl320_library import *
import time

# Define your settings here
# CONNECTION_DEVICE = "UART"    # change to "UART" if you are using UART connection
CONNECTION_DEVICE = "USB"   # change to "USB" if you are using USB2AX connection
# PORT_NAME = "/dev/ttyTHS1"    # UART has fixed port name ttyTHS1, 
PORT_NAME = "/dev/ttyACM2"  # USB port names are dynamic you need to check what it is

#SERVO 1 LIMIT: 0 - 1023 (0 - 300 deg)
#SERVO 2 LIMIT: 205 - 922 (60 - 270 deg)
#SERVO 3 LIMIT: 0 - 818 (0 - 240 deg)
#SERVO 4 LIMIT: 0 - 1023 (0 - 300 deg)
#SERVO 5 LIMIT: 205 - 818 (60 - 240 deg)

def degToCmd(deg):
    return int(3.41329*deg)

def cmdToDeg(cmd):
    return cmd/3.41329

def openGripper(servo):
    servo.set_position(degToCmd(120))
    
def closeGripper(servo):
    servo.set_position(degToCmd(165))

def idlePos(servo1, servo2, servo3, servo4, servo5):
    servo3.set_position(degToCmd(150))
    servo4.set_position(degToCmd(180))
    time.sleep(0.25)
    servo1.set_position(degToCmd(150))
    servo2.set_position(degToCmd(150))
    servo5.set_position(degToCmd(120))
    
def grabPos(servo1, servo2, servo3, servo4, servo5):
    servo3.set_position(degToCmd(80))
    time.sleep(0.5)
    servo1.set_position(degToCmd(60))
    servo2.set_position(degToCmd(150))
    servo4.set_position(degToCmd(185))
    time.sleep(1)
    servo3.set_position(degToCmd(120))
    time.sleep(0.75)
    closeGripper(servo5)
    
def idleHold(servo1, servo2, servo3, servo4, servo5):
    servo3.set_position(degToCmd(150))
    servo4.set_position(degToCmd(180))
    time.sleep(0.25)
    servo1.set_position(degToCmd(150))
    servo2.set_position(degToCmd(150))
    
def releasePos(servo1, servo2, servo3, servo4, servo5):
    servo2.set_position(degToCmd(150))
    servo3.set_position(degToCmd(60))
    servo4.set_position(degToCmd(150))
    time.sleep(1)
    servo1.set_position(degToCmd(150))
    time.sleep(1)
    servo3.set_position(degToCmd(0))
    servo4.set_position(degToCmd(120))
    time.sleep(1)
    openGripper(servo5)

def main():
    if CONNECTION_DEVICE == "UART":
        initialize_GPIO()
        portHandler, packetHandler = initialize_gpio_handlers(PORT_NAME)
    elif CONNECTION_DEVICE == "USB":
        portHandler, packetHandler = initialize_handlers(PORT_NAME)
    else:
        print("Invalid connnection device!")

    # defines the servo's ID
    servo1_ID = 1
    servo2_ID = 2
    servo3_ID = 3
    servo4_ID = 4
    servo5_ID = 5
    
    # define speed
    speed = 150

    open_port(portHandler)
    set_baudrate(portHandler, 1000000)

    servo1 = Servo(servo1_ID, portHandler, packetHandler)
    servo1.change_led_color(LED_RED)
    servo1.disable_torque()
    servo1.set_control_mode("joint")  # torque must be off when you change mode
    servo1.set_joint_speed(speed)
    servo1.enable_torque()

    servo2 = Servo(servo2_ID, portHandler, packetHandler)
    servo2.change_led_color(LED_YELLOW)
    servo2.disable_torque()
    servo2.set_control_mode("joint")  # torque must be off when you change mode
    servo2.set_joint_speed(speed)
    servo2.enable_torque()
    
    servo3 = Servo(servo3_ID, portHandler, packetHandler)
    servo3.change_led_color(LED_CYAN)
    servo3.disable_torque()
    servo3.set_control_mode("joint")  # torque must be off when you change mode
    servo3.set_joint_speed(speed - 50)
    servo3.enable_torque()

    servo4 = Servo(servo4_ID, portHandler, packetHandler)
    servo4.change_led_color(LED_PURPLE)
    servo4.disable_torque()
    servo4.set_control_mode("joint")  # torque must be off when you change mode
    servo4.set_joint_speed(speed)
    servo4.enable_torque()
    
    servo5 = Servo(servo5_ID, portHandler, packetHandler)
    servo5.change_led_color(LED_GREEN)
    servo5.disable_torque()
    servo5.set_control_mode("joint")  # torque must be off when you change mode
    servo5.set_joint_speed(speed)
    servo5.enable_torque()
    
#     servo1.disable_torque()
#     servo2.disable_torque()
#     servo3.disable_torque()
#     servo4.disable_torque()
#     servo5.disable_torque()

    #idle
    idlePos(servo1, servo2, servo3, servo4, servo5)
    time.sleep(2)
    
    #grab and hold
    grabPos(servo1, servo2, servo3, servo4, servo5)
    time.sleep(1)
    idleHold(servo1, servo2, servo3, servo4, servo5)
    time.sleep(2)
    
    #release and idle
    releasePos(servo1,servo2,servo3,servo4,servo5)
    time.sleep(1)
    idlePos(servo1, servo2, servo3, servo4, servo5)
     
    print(servo1.get_position(),' ',servo2.get_position(),' ',servo3.get_position(),' ',
               servo4.get_position(), ' ', servo5.get_position())

    


        
if __name__ == "__main__":
    main()