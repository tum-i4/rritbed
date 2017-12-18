#!/usr/bin/env python
""" Repeated mover - indefinitely repeats basic mover """

import basic_mover
import rospy


def repeat_move():
	""" Indefinitely move robot according to movement file """
	while not rospy.is_shutdown():
		basic_mover.move()


if __name__ == "__main__":
	try:
		repeat_move()
	except rospy.ROSInterruptException:
		pass
