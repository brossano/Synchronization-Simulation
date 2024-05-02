# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Int32

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import time

from enum import Enum

class AMRState(Enum):
    NAV_TO_RACK = 0,
    AT_RACK = 1,
    NAV_TO_AGV = 2,
    SYNC = 3,
    DONE = 4

class AMRNav(Node):

    def __init__(self):
        super().__init__('amr_nav')

        time.sleep(5)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.sync_publisher_ = self.create_publisher(Bool, '/sync', 10)

        self.vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.manipulator_publisher_ = self.create_publisher(Int32, '/manipulator_state', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.amr_state = AMRState.NAV_TO_RACK

        self.navigator = BasicNavigator()


    def timer_callback(self):
        from_frame_rel = 'base_footprint'
        to_frame_rel = 'map'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = t.transform.translation.x
        initial_pose.pose.position.y = t.transform.translation.y
        initial_pose.pose.orientation.z = t.transform.rotation.z
        initial_pose.pose.orientation.w = t.transform.rotation.w
        self.navigator.setInitialPose(initial_pose)

        # x = t.transform.rotation.x
        # yaw = math.atan2(2.0*(t.transform.rotation.y*t.transform.rotation.z + t.transform.rotation.w*t.transform.rotation.x), 
        #                 t.transform.rotation.w*t.transform.rotation.w - t.transform.rotation.x*t.transform.rotation.x - 
        #                 t.transform.rotation.y*t.transform.rotation.y + t.transform.rotation.z*t.transform.rotation.z)


        
        if self.amr_state==AMRState.NAV_TO_RACK:
            goal_x = 1.3
            goal_y = -0.2
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            # goal_pose.pose.position.x = t.transform.translation.x + 0.3*math.sin(yaw)
            # goal_pose.pose.position.y =  t.transform.translation.y - 0.3*math.cos(yaw)
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y =  goal_y
            goal_pose.pose.orientation.w = 1.0

            # print(goal_x, goal_y)

            # path = self.navigator.getPath(initial_pose, goal_pose)

            # if not path:
            #     goal_dir_x = goal_x - initial_pose.pose.position.x
            #     goal_dir_y = goal_y - initial_pose.pose.position.y

            #     goal_pose.pose.position.x = initial_pose.pose.position.x + 1/4*goal_dir_x
            #     goal_pose.pose.position.y = initial_pose.pose.position.y + 1/4*goal_dir_y


            self.navigator.goToPose(goal_pose)

            feedback = self.navigator.getFeedback()
            if feedback:
                print(feedback.distance_remaining)

        
        if self.navigator.isTaskComplete() and self.amr_state==AMRState.NAV_TO_RACK:
            self.amr_state = AMRState.AT_RACK

        if self.amr_state==AMRState.AT_RACK:
            print("Performing manipulation at rack")
            manipulator_state = Int32()
            manipulator_state.data = 0
            self.manipulator_publisher_.publish(manipulator_state)
            time.sleep(7)
            cmd_vel = Twist()
            cmd_vel.linear.x = -0.2
            self.vel_publisher_.publish(cmd_vel)
            time.sleep(8)
            cmd_vel.linear.x = 0.0
            self.vel_publisher_.publish(cmd_vel)
            self.amr_state=AMRState.NAV_TO_AGV
            time.sleep(3)


        if self.amr_state==AMRState.NAV_TO_AGV:
            goal_x = 2.6
            goal_y = -0.8
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            # goal_pose.pose.position.x = t.transform.translation.x + 0.3*math.sin(yaw)
            # goal_pose.pose.position.y =  t.transform.translation.y - 0.3*math.cos(yaw)
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y =  goal_y
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.w = 1.0
            
            # path = self.navigator.getPath(initial_pose, goal_pose)

            # if not path:
            #     goal_pose.pose.position.x = goal_x/2
            #     goal_pose.pose.position.y = goal_y/2


            self.navigator.goToPose(goal_pose)

            feedback = self.navigator.getFeedback()
            if feedback:
                print(feedback.distance_remaining)

        if self.navigator.isTaskComplete() and self.amr_state==AMRState.NAV_TO_AGV:
            self.amr_state = AMRState.SYNC

        if self.amr_state == AMRState.SYNC:
            sync = Bool()
            sync.data = True
            self.sync_publisher_.publish(sync)
            self.amr_state = AMRState.DONE






def main(args=None):
    rclpy.init(args=args)

    amr_nav = AMRNav()

    rclpy.spin(amr_nav)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# def main():
#     rclpy.init()

#     tf_buffer = Buffer()
#     tf_listener = TransformListener(tf_buffer, True)
#     from_frame_rel = 'map'
#     to_frame_rel = 'tb3_1/base_footprint'
#     t = tf_buffer.lookup_transform(
#                 to_frame_rel,
#                 from_frame_rel,
#                 rclpy.time.Time())
#     print(t)
#     navigator = BasicNavigator()

#     # Set our demo's initial pose
#     # initial_pose = PoseStamped()
#     # initial_pose.header.frame_id = 'map'
#     # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
#     # initial_pose.pose.position.x = 2.0
#     # initial_pose.pose.position.y = 1.7
#     # initial_pose.pose.orientation.z = 1.0
#     # initial_pose.pose.orientation.w = 0.0
#     # navigator.setInitialPose(initial_pose)

#     # Activate navigation, if not autostarted. This should be called after setInitialPose()
#     # or this will initialize at the origin of the map and update the costmap with bogus readings.
#     # If autostart, you should `waitUntilNav2Active()` instead.
#     # navigator.lifecycleStartup()

#     # Wait for navigation to fully activate, since autostarting nav2
#     # navigator.waitUntilNav2Active()

#     # If desired, you can change or load the map as well
#     # navigator.changeMap('/path/to/map.yaml')

#     # You may use the navigator to clear or obtain costmaps
#     # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
#     # global_costmap = navigator.getGlobalCostmap()
#     # local_costmap = navigator.getLocalCostmap()

#     # Go to our demos first goal pose
#     goal_pose = PoseStamped()
#     goal_pose.header.frame_id = 'map'
#     goal_pose.header.stamp = navigator.get_clock().now().to_msg()
#     goal_pose.pose.position.x = t.transform.translation.x
#     goal_pose.pose.position.y =  t.transform.translation.y
#     goal_pose.pose.orientation.w = t.transform.rotation.w

#     # sanity check a valid path exists
#     # path = navigator.getPath(initial_pose, goal_pose)

#     navigator.goToPose(goal_pose)

#     i = 0
#     while not navigator.isTaskComplete():
#         ################################################
#         #
#         # Implement some code here for your application!
#         #
#         ################################################

#         # Do something with the feedback
#         i = i + 1
#         feedback = navigator.getFeedback()
#         print(feedback.distance_remaining)
#         if feedback and i % 5 == 0:
#             print(
#                 'Estimated time of arrival: '
#                 + '{0:.0f}'.format(
#                     Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
#                     / 1e9
#                 )
#                 + ' seconds.'
#             )

#             # Some navigation timeout to demo cancellation
#             if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
#                 navigator.cancelTask()

#             # Some navigation request change to demo preemption
#             if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
#                 goal_pose.pose.position.x = -3.0
#                 navigator.goToPose(goal_pose)

#     # Do something depending on the return code
#     result = navigator.getResult()
#     if result == TaskResult.SUCCEEDED:
#         print('Goal succeeded!')
#     elif result == TaskResult.CANCELED:
#         print('Goal was canceled!')
#     elif result == TaskResult.FAILED:
#         print('Goal failed!')
#     else:
#         print('Goal has an invalid return status!')

#     navigator.lifecycleShutdown()

#     exit(0)


# if __name__ == '__main__':
#     main()