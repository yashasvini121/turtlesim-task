#!/usr/bin/env python3

import rospy
from turtle_mover import TurtleMover
from math import pi

class MoveCircle(TurtleMover):
    def __init__(self, *, speed=1.0, radius=1.0):
        """
        Derived class to move the turtle in a circle.
        """
        self.radius = radius
        self.speed = speed
        angular_speed = self.speed / self.radius  # Calculate angular speed for circular motion
        super().__init__((self.speed, 0.0, 0.0), (0.0, 0.0, angular_speed))

    def move_circular(self):
        """
        move_circular(): Move the turtle in a circle by setting linear and angular velocities.
        :status: to_fix
        :issue: The duration (time) calculation is not accurate.
        """
        # Set velocities for circular motion
        self.set_velocity(linear_x=self.speed, angular_z=self.speed / self.radius)
        
        # Calculate the duration to complete one full circle
        # TODO: BUG: Formula not giving accurate results, so using a constant value
        duration = (2 * pi * self.radius) / self.speed
        self.move_for_duration(duration)


if __name__ == '__main__':
    try:
        circle_mover = MoveCircle(radius=2)
        print(f"\033[33m{circle_mover.move_circular.__doc__}\033[33m")
        circle_mover.reset_turtlesim(0)
        circle_mover.move_circular()
    except rospy.ROSInterruptException:
        pass
