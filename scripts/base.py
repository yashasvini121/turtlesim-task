#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from abc import ABC, abstractmethod
import time
from std_srvs.srv import Empty

class TurtleMover(ABC):
    def __init__(self, linear_speed=1.0, angular_speed=0.5):
        """
        Set up the ROS node, create a publisher, and initialize common properties for turtle movement.
        """
        rospy.init_node('move_turtle_node', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.vel_msg = Twist()

        # Common movement properties
        self.linear_speed = 1.0    # Default linear speed (m/s)
        self.angular_speed = 0.5   # Default angular speed (rad/s)

        # Initialize default velocities
        self.set_velocity(0.0, 0.0)

    def set_velocity(self, linear_x, angular_z):
        """
        Set the linear and angular velocities of the turtle.
        """
        self.vel_msg.linear.x = linear_x
        self.vel_msg.angular.z = angular_z

    def publish_velocity(self):
        """
        Publish the current velocity message and wait for the specified rate.
        """
        self.velocity_publisher.publish(self.vel_msg)
        self.rate.sleep()

    # @abstractmethod
    # def move(self):
    #     """
    #     Abstract method to be implemented by subclasses for specific movement patterns.
    #     """
    #     pass

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
        self.set_velocity(0.0, 0.0)
        self.publish_velocity()

    def end_and_clear_rospy(self):
        """
        End the ROS node and clear the rospy context.
        """
        rospy.signal_shutdown("Shutting down...")
        rospy.loginfo("ROS node shutdown")


    def reset_turtlesim():
        rospy.init_node('reset_turtlesim_node')
        rospy.wait_for_service('/reset')
        try:
            reset_service = rospy.ServiceProxy('/reset', Empty)
            reset_service()
            print("Turtlesim screen reset successfully.")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    
    def get_to_coordinates(self, x, y):
        """
        Move the turtle to the specified coordinates (x, y).
        """
        self.set_velocity(self.linear_speed, 0.0)
        while not rospy.is_shutdown():
            self.publish_velocity()
            if round(self.x, 2) == x and round(self.y, 2) == y:
                break
        self.stop()

    # def __del__(self):
    #     """
    #     Destructor to stop the turtle and clear the rospy context.
    #     """
    #     self.stop()
    #     self.end_and_clear_rospy()
    
    def change_direction(self, angle, clockwise=False):
        """
        Change the direction of the turtle based on the input direction in radians.
        """
        angular_speed = self.angular_speed if clockwise else -self.angular_speed
        start_time = time.time()
        while time.time() - start_time < abs(angle) / abs(self.angular_speed) and not rospy.is_shutdown():
            self.set_velocity(0.0, angular_speed)
            self.publish_velocity()
        self.stop()

    def move_straight(self, distance):
        """
        Move the turtle in a straight line for the specified distance.
        """
        start_time = time.time()
        while time.time() - start_time < distance / self.linear_speed and not rospy.is_shutdown():
            self.set_velocity(self.linear_speed, 0.0)
            self.publish_velocity()
        self.stop()
