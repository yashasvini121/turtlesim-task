#!/usr/bin/env python3

import rospy
from turtle_mover import TurtleMover
import sys

class MoveStraight(TurtleMover):
    def __init__(self, linear_speed=(1.0, 0.0, 0.0), angular_speed=(0.0, 0.0, 0.0)):
        """
        Derived class to move the turtlebot in a straight line.
        Not keeping distance and axis as arguments because they aren't required for long-term use.
        """
        super().__init__(linear_speed, angular_speed)

    def move_turtle_straight(self, distance=2.0, axis='x'):
        self.reset_turtlesim(0)
        self.move_straight(distance, axis)

if __name__ == '__main__':
    if len(sys.argv) > 3:
        print("\033[31mUsage: rosrun ros_session move_straight.py optional: <distance:float 2.0> <axis:str 'x'>\033[31m")
        print("\033[31mLinear and Angular Speeds are not added in arguements yet\033[31m")
    else:
        try:
            # User Input
            distance = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
            axis = sys.argv[2] if len(sys.argv) > 2 else 'x'

            straight_mover = MoveStraight()
            straight_mover.move_turtle_straight(distance, axis)
        except rospy.ROSInterruptException:
            pass
