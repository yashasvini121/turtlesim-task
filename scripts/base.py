#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from abc import ABC, abstractmethod
import time
from std_srvs.srv import Empty
import math

class TurtleMover(ABC):
    def __init__(self, linear_speed: tuple, angular_speed: tuple):
        """
        Set up the ROS node, create a publisher, and initialize common properties for turtle movement.
        """
        rospy.init_node('move_turtle_node', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.vel_msg = Twist()

        # Store the speed as tuples
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

        # Initialize the Twist message with the given speed components
        self.vel_msg.linear.x, self.vel_msg.linear.y, self.vel_msg.linear.z = self.linear_speed
        self.vel_msg.angular.x, self.vel_msg.angular.y, self.vel_msg.angular.z = self.angular_speed

    def set_velocity(self, linear_x=None, linear_y=None, linear_z=None, angular_x=None, angular_y=None, angular_z=None):
        """
        Set the linear and angular velocities of the turtle.
        """
        self.vel_msg.linear.x = linear_x if linear_x is not None else self.vel_msg.linear.x
        self.vel_msg.linear.y = linear_y if linear_y is not None else self.vel_msg.linear.y
        self.vel_msg.linear.z = linear_z if linear_z is not None else self.vel_msg.linear.z
        self.vel_msg.angular.x = angular_x if angular_x is not None else self.vel_msg.angular.x
        self.vel_msg.angular.y = angular_y if angular_y is not None else self.vel_msg.angular.y
        self.vel_msg.angular.z = angular_z if angular_z is not None else self.vel_msg.angular.z

    def publish_velocity(self):
        """
        Publish the current velocity message and wait for the specified rate.
        """
        self.velocity_publisher.publish(self.vel_msg)
        self.rate.sleep()

    def move_for_duration(self, duration):
        """
        Move the turtle at the current velocity for a specified duration.
        """
        start_time = time.time()
        while time.time() - start_time < duration and not rospy.is_shutdown():
            self.publish_velocity()

    def stop(self):
        """
        Stop the turtle by setting velocities to zero and publishing.
        """
        self.set_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.publish_velocity()

    def end_and_clear_rospy(self):
        """
        End the ROS node and clear the rospy context.
        """
        pass

    def reset_turtlesim(self, seconds=2):
        """
        Reset the turtlesim screen after the specified number of seconds.
        """
        rospy.sleep(seconds)

        rospy.wait_for_service('/reset')
        try:
            reset_service = rospy.ServiceProxy('/reset', Empty)
            reset_service()
            rospy.loginfo("Turtlesim screen reset successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
    def get_to_coordinates(self, x, y):
        """
        Move the turtle to the specified coordinates (x, y).
        """
        # Assuming initial position (0,0)
        self.x, self.y = 0.0, 0.0

        # Calculate the angle to the target coordinates
        angle_to_target = math.atan2(y - self.y, x - self.x)
        self.rotate(angle_to_target)

        # Calculate the distance to the target coordinates
        distance_to_target = math.sqrt((x - self.x) ** 2 + (y - self.y) ** 2)

        # Move the turtle to the target coordinates
        self.move_straight(distance_to_target)

    def rotate(self, angle:float, clockwise:bool = False, angular_speed:float = 1.0):
        """
        Rotate the turtle by the specified angle in the specified direction.
        """
        # Stop turtle before changing direction
        self.stop()

        self.vel_msg.angular.z = -angular_speed if clockwise else angular_speed
        start_time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - start_time < abs(angle) / abs(angular_speed) and not rospy.is_shutdown():
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()
        
        # Stop turning
        self.vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.vel_msg)


    def move_straight(self, distance, axis_index=0):
        """
        Move the turtle in a straight line for the specified distance.
        Axis index: 0 for x-axis, 1 for y-axis, 2 for z-axis.
        """
        start_time = time.time()
        while time.time() - start_time < distance / abs(self.linear_speed[axis_index]) and not rospy.is_shutdown():
            self.set_velocity(linear_x=self.linear_speed[axis_index])
            self.publish_velocity()

# Example usage
if __name__ == "__main__":
    mover = TurtleMover(linear_speed=(1.0, 0.0, 0.0), angular_speed=(0.0, 0.0, 0.0))
    mover.move_straight(2)
    mover.rotate(math.pi / 2, clockwise=False, angular_speed=1)
    mover.move_straight(1)
    mover.reset_turtlesim()