#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class FilterNode:

    twist_pub = None
    twist_sub = None

    last_twist = Twist()

    def __init__(self):
        self.twist_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
        self.twist_sub = rospy.Subscriber('autonomous_driver/drive_cmd', Twist, self.autonomous_command_callback)

    def publish(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.twist_pub.publish(self.last_twist)
            rate.sleep()

    def autonomous_command_callback(self, twist):
        self.last_twist = twist
        self.twist_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('filter', log_level=rospy.DEBUG)

    node = FilterNode()
    node.publish()
    
    rospy.spin()