#!/usr/bin/env python3

import rospy
from turtle_mover import TurtleMover
from math import pi
import sys

class MoveCircle(TurtleMover):
    def __init__(self):
        """
        Derived class to move the turtle in a circle.
        """
        self.radius = radius
        super().__init__()

    def move_circular(self, radius:float=1.0, speed:float=1.0):
        """
        move_circular(): Move the turtle in a circle by setting linear and angular velocities.
        Not keeping radius and speed as class variables seem to be a better approach.
        :status: to_fix
        :issue: The duration (time) calculation is not accurate.
        """
        # Set velocities for circular motion
        self.set_velocity(linear_x=speed, angular_z=speed/radius)
        
        # Calculate the duration to complete one full circle
        # TODO: BUG: Formula not giving accurate results, so using a constant value
        duration = (2 * pi * radius) / speed
        self.move_for_duration(duration)


if __name__ == '__main__':
    if len(sys.argv) > 3:
        print("\033[31mUsage: rosrun ros_session move_circle.py optional: <radius:float 1> <speed:float 1>\033[31m")
    else:
        try:
            # User Input
            radius = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
            speed = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

            circle_mover = MoveCircle()
            print(f"\033[33m{circle_mover.move_circular.__doc__}\033[33m")
            circle_mover.reset_turtlesim(0)
            circle_mover.move_circular(radius, speed)
            
        except rospy.ROSInterruptException:
            pass
