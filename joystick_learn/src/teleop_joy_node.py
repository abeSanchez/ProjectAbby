#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

LINEAR_AXIS = 5
ANGULAR_AXIS = 0
    
LINEAR_SCALE = 1
ANGULAR_SCALE = 1

GATE_THRESH = 0.1

class TeleopJoyNode:
    
    twist_pub = None
    joy_sub = None
    last_twist = Twist()

    def __init__(self):
        self.twist_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

    def publish(self):
        while not rospy.is_shutdown():
            self.twist_pub.publish(self.last_twist)

    def joy_callback(self, joy_msg):
        twist = Twist()

        linear_x = -(joy_msg.axes[LINEAR_AXIS] - 1) / 2
        angular_z = ANGULAR_SCALE * joy_msg.axes[ANGULAR_AXIS]

        if linear_x > GATE_THRESH:
            twist.linear.x = linear_x
        
        if abs(angular_z) > GATE_THRESH:
            twist.angular.z = angular_z
        
        self.last_twist = twist
        self.twist_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('teleop_joystick', log_level=rospy.DEBUG)

    node = TeleopJoyNode()
    node.publish()
    
    rospy.spin()