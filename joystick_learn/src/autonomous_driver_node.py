#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from enum import Enum

class Modes(Enum):
    USER_MODE = 1
    CRUISE_MODE = 2
    AUTOPILOT_MODE = 3
    DATA_COLLECTION_MODE = 4

class Autononmous:
    
    selected_mode = Modes.USER_MODE

    twist_pub = None
    twist_sub = None
    ultrasonic_sub = None
    depth_camera_sub = None

    last_twist = Twist()

    def __init__(self):
        self.twist_pub = rospy.Publisher('autonomous_driver/drive_cmd', Twist, queue_size=1)
        self.twist_sub = rospy.Subscriber('joystick/drive_cmd', Twist, self.joystick_command_callback)

    def publish(self):
        while not rospy.is_shutdown():
            if self.selected_mode == Modes.USER_MODE:
                self.twist_pub.publish(self.last_twist)

    # Pass through drive commands if in USER_MODE
    def joystick_command_callback(self, twist):
        twist = Twist()
        
        if self.selected_mode == Modes.USER_MODE:
            self.last_twist = twist
            self.twist_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('autonomous_driver', log_level=rospy.DEBUG)

    node = AutononmousDriverNode()
    node.publish()
    
    rospy.spin()