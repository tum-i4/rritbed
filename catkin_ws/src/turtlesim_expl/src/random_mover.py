#!/usr/bin/env python
""" Basic mover """

import random
import time
import rospy
from geometry_msgs.msg import Twist

import move_helper

PI = 3.1415926535897

def rand_move():
	""" Move robot randomly """

	# Start a new node
	rospy.init_node("robot_mover", anonymous=True)
	velocity_publisher = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)

	rospy.loginfo("Starting random walker")

	# While loop to assure that Ctrl-C can exit the app
	while not rospy.is_shutdown():
		vel_msg = move_helper.get_zero_twist()

		# Decide if the turtle walks or turns
		turtle_walks = random.choice([True, False])

		# Velocity should be between -10 and 10, linear x (walk) or angular z (turn)
		veloc_value = random.choice(range(-10, 11))

		if turtle_walks:
			vel_msg.linear.x = veloc_value
		else:
			vel_msg.angular.z = veloc_value

		velocity_publisher.publish(vel_msg)
		# TODO: Try cleaner solution
		time.sleep(0.5)

    # Make sure to stop robot after the program has been cancelled
	velocity_publisher.publish(move_helper.get_zero_twist())


if __name__ == "__main__":
	try:
		rand_move()
	except rospy.ROSInterruptException:
		pass
