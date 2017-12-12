#!/usr/bin/env python
""" Basic mover """

import os
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897


def move():
    """ Move robot according to movement file """

    # Start a new node
    rospy.init_node('robot_mover', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    file_path = os.path.join(os.getcwd(), "move")
    if not os.path.isfile():
        print("No movement file found")
        return

    current_line = 0
    file_contents = []

    # Try to read the whole movement file
    try:
        file_reader = open(file_path)
        file_contents = file_reader.readlines()
        file_reader.close()
    except IOError:
        print("Couldn't read movement file")
        return

    # While loop to assure that Ctrl-C can exit the app
    while not rospy.is_shutdown():
        vel_msg = get_zero_twist()

        if current_line >= len(file_contents):
            print("End of movement file reached")
            return
        
        # Read next movement command
        # ...

    pass


def get_zero_twist():
    """ Return new twist initialised to zero """

    new_twist = Twist()

    new_twist.linear.x = 0
    new_twist.linear.y = 0
    new_twist.linear.z = 0
    new_twist.angular.x = 0
    new_twist.angular.y = 0
    new_twist.angular.z = 0

    return new_twist

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
