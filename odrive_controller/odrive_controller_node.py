# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
import odrive
from odrive.enums import *


from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

PI = 3.14159265358979323846
WHEEL_DIAMETER = 0.17
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_DISTANCE = 0.4
TURNUNG_RADIUS = WHEEL_DISTANCE / 2
WHEEL_CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS
TURNING_CIRCUMFERENCE = 2 * PI * TURNUNG_RADIUS

class controller_node(Node):

    def __init__(self):
        super().__init__('odrive_controller')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.odrv0 = odrive.find_any()
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.input_vel = 0
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.controller.input_vel = 0


        self.publisher_ = self.create_publisher(JointState, 'odrive_state', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Connected and publishing odrive_state')

    def listener_callback(self, msg):

        self.get_logger().info('-----------------------------------------------------------------------')
        self.get_logger().info('I heard in x: "%s"' % msg.linear.x)
        self.get_logger().info('I heard in z: "%s"' % msg.angular.z)
        self.get_logger().info('-----------------------------------------------------------------------')


        # linear speed calculations
        speed_x = msg.linear.x / WHEEL_CIRCUMFERENCE

        # angular speed calculations
        if(msg.angular.z < 0):
            speed_ang = -msg.angular.z
            dir = 1
        else:
            speed_ang = msg.angular.z
            dir = -1

        speed_ang_m_per_s = speed_ang * TURNING_CIRCUMFERENCE / (2 * PI)

        speed_left = speed_x + dir*speed_ang_m_per_s
        speed_right = speed_x - dir*speed_ang_m_per_s

        

        self.odrv0.axis0.controller.input_vel = speed_right
        self.odrv0.axis1.controller.input_vel = -speed_left

    def timer_callback(self):
        msg = JointState()

        msg.name = ["right_wheel_joint", "left_wheel_joint"]
        msg.position = [self.odrv0.axis0.encoder.pos_estimate,self.odrv0.axis1.encoder.pos_estimate]
        msg.velocity = [self.odrv0.axis0.encoder.vel_estimate,self.odrv0.axis1.encoder.vel_estimate]
        msg.effort = [self.odrv0.vbus_voltage,self.odrv0.vbus_voltage]

        self.odrv0.axis0.error = 0
        self.odrv0.axis1.error = 0
        self.odrv0.axis0.watchdog_feed()
        self.odrv0.axis1.watchdog_feed()
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    controller_node_ = controller_node()

    rclpy.spin(controller_node_)

    controller_node_.odrv0.axis0.controller.input_vel = 0
    controller_node_.odrv0.axis1.controller.input_vel = 0

    controller_node_.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
