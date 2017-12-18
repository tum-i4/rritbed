#!/usr/bin/env python
""" Logging node """

import rospy
from std_msgs.msg import String


def log(data):
	""" TODO currently logs, should call API """

	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def logger():
	""" Main function """
	rospy.init_node('logger', anonymous=True)

	rospy.Subscriber("chatter", String, log)

	# spin() keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	logger()
