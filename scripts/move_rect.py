#!/usr/bin/env python3

import rospy
import sys
from turtle_mover import TurtleMover
from ast import literal_eval

class MoveRectangle(TurtleMover):
    """:status: pending  The direction feature is not implemented yet."""
    
    def __init__(self, length_, breadth, speed=1.0, clockwise=True, facing_direction='w'):
        super().__init__((speed, 0, 0), (0, 0, 0))
        self.length = length_
        self.breadth = breadth
        self.set_velocity(linear_x=speed)
        self.clockwise = clockwise
        self.facing_direction = facing_direction

    def move_rect(self):
        """
        Move the turtle in a rectangle path.
        """
        speed = self.vel_msg.linear.x

        if self.clockwise:
            self.move_straight(length, 'x')

            self.set_velocity(linear_x=0, linear_y=-speed)
            self.move_straight(self.breadth, 'y')

            self.set_velocity(linear_x=-speed, linear_y=0)
            self.move_straight(self.length, 'x')

            self.set_velocity(linear_x=0, linear_y=speed)
            self.move_straight(self.breadth, 'y')
        else:
            self.move_straight(self.length, 'x')

            self.set_velocity(linear_x=0, linear_y=speed)
            self.move_straight(self.breadth, 'y')

            self.set_velocity(linear_x=-speed, linear_y=0)
            self.move_straight(self.length, 'x')

            self.set_velocity(linear_x=0, linear_y=-speed)
            self.move_straight(self.breadth, 'y')

if __name__ == '__main__':
    if len(sys.argv) > 6:
        print("\033[31mUsage: rosrun ros_session move_rect.py <length> <breadth> optional: <speed:float 1> <clockwise:bool TRUE> <facing_direction:char N [n, w, e, s]>\033[31m")
    else:
        try:
            # User Input
            length = float(sys.argv[1])
            breadth = float(sys.argv[2])
            speed = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
            clockwise = literal_eval((sys.argv[4]).capitalize()) if len(sys.argv) > 4 else True
            facing_direction = sys.argv[5].lower() if len(sys.argv) > 5 else 'w'
            
            move_rect = MoveRectangle(length, breadth, speed, clockwise)
            move_rect.reset_turtlesim()
            move_rect.move_rect()
        except rospy.ROSInterruptException:
            pass
