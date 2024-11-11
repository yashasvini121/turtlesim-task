#!/usr/bin/env python3

import rospy
from turtle_mover import TurtleMover

class MoveStraight(TurtleMover):
    def __init__(self):
        """
        Derived class to move the turtlebot in a straight line.
        """
    
        super().__init__((0.0, 1.0, 0.0), (0.0, 0.0, 0.0))

    def move_turtle_straight(self):
        self.reset_turtlesim(0)
        self.move_straight(4)

if __name__ == '__main__':
    try:
        straight_mover = MoveStraight()
        straight_mover.move_turtle_straight()
    except rospy.ROSInterruptException:
        pass
