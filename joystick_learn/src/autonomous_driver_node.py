#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from enum import Enum
from keras.models import model_from_json
import os

class Modes(Enum):
    USER_MODE = 1
    CRUISE_MODE = 2
    AUTOPILOT_MODE = 3
    DATA_COLLECTION_MODE = 4

class AutononmousDriverNode:
    
    selected_mode = Modes.USER_MODE

    twist_pub = None
    twist_sub = None
    ultrasonic_sub = None
    depth_camera_sub = None

    blocked_model = None
    orient_model = None

    last_twist = Twist()

    def __init__(self):
        self.set_pubs_and_subs()
        self.load_models()

    def set_pubs_and_subs(self):
        self.twist_pub = rospy.Publisher('autonomous_driver/drive_cmd', Twist, queue_size=10)
        self.twist_sub = rospy.Subscriber('joystick/drive_cmd', Twist, self.joystick_command_callback)
        self.ultrasonic_sub = rospy.Subscriber('arduino/ultrasonic_ranges', Float32MultiArray, self.perimeter_check)

    def load_models():
        # Load blocked model
        json_file = open('blocked_model.json', 'r')
        loaded_model_json = json_file.read()
        self.blocked_model = model_from_json(loaded_model_json)
        self.blocked_model.load_weights('blocked_model.h5')

        # Load orient model
        json_file = open('orient_model.json', 'r')
        loaded_model_json = json_file.read()
        self.orient_model = model_from_json(loaded_model_json)
        self.orient_model.load_weights('orient_model.h5')

    def perimeter_check(self, ultrasonic_ranges):
        if not self.perimeter_clear(ultrasonic_ranges):
            self.selected_mode = Modes.USER_MODE

    def perimeter_clear(self, ultrasonic_ranges):
        for r in ultrasonic_ranges:
            if r < ULTRASONIC_THRESH:
                return False
        return True

    def publish(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.selected_mode == Modes.USER_MODE:
                self.twist_pub.publish(self.last_twist)
            rate.sleep()

    # Pass through drive commands if in USER_MODE
    def joystick_command_callback(self, twist):
        if self.selected_mode == Modes.USER_MODE:
            self.last_twist = twist
            self.twist_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('autonomous_driver', log_level=rospy.DEBUG)

    node = AutononmousDriverNode()
    node.publish()
    
    rospy.spin()