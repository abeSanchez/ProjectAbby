#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from enum import IntEnum

LINEAR_AXIS = 5
ANGULAR_AXIS = 0
    
LINEAR_SCALE = 1
ANGULAR_SCALE = 1

GATE_THRESH = 0.1

class Modes(IntEnum):
    USER_MODE = 1
    CRUISE_MODE = 2
    AUTOPILOT_MODE = 3
    DATA_COLLECTION_MODE = 4

class TeleopJoyNode:
    
    twist_pub = None
    mode_pub = None
    joy_sub = None
    last_twist = Twist()

    def __init__(self):
        self.twist_pub = rospy.Publisher('joystick/drive_cmd', Twist, queue_size=10)
        self.mode_pub = rospy.Publisher('joystick/mode', Int32, queue_size=10)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

    def publish(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.twist_pub.publish(self.last_twist)
            rate.sleep()

    def joy_callback(self, joy_msg):
        twist = Twist()
        mode = Int32()

        if joy_msg.buttons[0] == 1:
            mode.data = int(Modes.AUTOPILOT_MODE)
            self.mode_pub.publish(mode)
        if joy_msg.buttons[2] == 1:
            mode.data = int(Modes.USER_MODE)
            self.mode_pub.publish(mode)

        linear_x = -(joy_msg.axes[LINEAR_AXIS] - 1) / 2
        angular_z = ANGULAR_SCALE * joy_msg.axes[ANGULAR_AXIS]

        if linear_x > GATE_THRESH:
            twist.linear.x = linear_x
        
        if abs(angular_z) > GATE_THRESH:
            twist.angular.z = angular_z
        
        self.last_twist = twist

if __name__ == '__main__':
    rospy.init_node('teleop_joystick', log_level=rospy.DEBUG)

    node = TeleopJoyNode()
    node.publish()
    
    rospy.spin()