#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3, Pose
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
        # self._pose_turtle = Pose()

        # Store the speeds as Vector3 objects
        self.vel_msg.linear = Vector3(*linear_speed)
        self.vel_msg.angular = Vector3(*angular_speed)

    @property
    def linear_speed(self):
        return (self.vel_msg.linear.x, self.vel_msg.linear.y, self.vel_msg.linear.z)
    
    @property
    def angular_speed(self):
        return (self.vel_msg.angular.x, self.vel_msg.angular.y, self.vel_msg.angular.z)
    
    def reset_turtlesim(self, seconds=2):
        """
        Reset the turtlesim screen after the specified number of seconds.
        """
        rospy.sleep(seconds)

        rospy.wait_for_service('/reset')
        try:
            reset_service = rospy.ServiceProxy('/reset', Empty)
            reset_service()
            # rospy.loginfo("Turtlesim screen reset successfully.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def end_and_clear_rospy(self):
        """
        End the ROS node and clear the rospy context.
        Is this even required?
        """
        pass

    def _publish_velocity(self):
        """
        Publish the current velocity message and wait for the specified rate.
        """
        self.velocity_publisher.publish(self.vel_msg)
        self.rate.sleep()

    def set_velocity(self, /, *, speed_twist_obj:Twist=None, linear_x:float=None, linear_y:float=None, linear_z:float=None, angular_x:float=None, angular_y:float=None, angular_z:float=None):
        """
        Set the linear and angular velocities of the turtle.
        Arguments are passed as keyword arguments. 
        Only updates the velocity components that are provided.
        """
        # Set the velocity message to the provided Twist object
        if speed_twist_obj is not None:
            self.vel_msg = speed_twist_obj
            self._publish_velocity()
            return
        
        # Set linear velocities only for the arguments that are present
        if linear_x is not None:
            self.vel_msg.linear.x = linear_x
        if linear_y is not None:
            self.vel_msg.linear.y = linear_y
        if linear_z is not None:
            self.vel_msg.linear.z = linear_z

        # Set angular velocities only for the arguments that are present
        if angular_x is not None:
            self.vel_msg.angular.x = angular_x
        if angular_y is not None:
            self.vel_msg.angular.y = angular_y
        if angular_z is not None:
            self.vel_msg.angular.z = angular_z

        # Publish the updated velocity message
        self._publish_velocity()
    
    def get_axis_speed(self, axis='x'):
        """
        Get the current speed of the turtle along the specified axis.
        """
        return getattr(self.vel_msg.linear, axis)

    def stop(self):
        """
        Stop the turtle by setting velocities to zero and publishing.
        """
        self.set_velocity(linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0)

    # def move(self, /, *, speed_twist_obj:Twist=None, linear_x:float=1.0, linear_y:float=0.0, linear_z:float=0.0):
    #     """
    #     Set velocity to move the turtle in the specified direction.
    #     Though the function is named 'move', it only sets the velocity. So commenting out this function.
    #     """
    #     self.set_velocity(speed_twist_obj=speed_twist_obj, linear_x=linear_x, linear_y=linear_y, linear_z=linear_z)

    def move_for_duration(self, duration):
        """
        Move the turtle at the current velocity for a specified duration.
        """
        start_time = time.time()
        self.set_velocity(linear_x=1.0)
        while time.time() - start_time < duration and not rospy.is_shutdown():
            self._publish_velocity()

    # def get_to_coordinates(self, x, y):
    #     """
    #     Move the turtle to the specified coordinates (x, y).
    #     """
    #     # Check velocity components
    #     if self.vel_msg.linear == Vector3(0.0, 0.0, 0.0):
    #         rospy.logwarn("Cannot move to coordinates with non-zero y or z velocity components.")
    #         rospy.logwarn("Setting default velocities")
    #         self.set_velocity(linear_x=1.0)
        
    #     # Get current coordinates
    #     # self._pose_turtle = Point()
    #     curr_x = self._pose_turtle.x
    #     curr_y = self._pose_turtle.y
    #     print(f"Current coordinates: ({curr_x}, {curr_y})")

    #     # Calculate the angle to the target coordinates
    #     angle_to_target = math.atan2(y - curr_y, x - curr_x)
    #     self.rotate(angle_to_target)

    #     # Calculate the distance to the target coordinates
    #     distance_to_target = math.sqrt((x - curr_x) ** 2 + (y - curr_y) ** 2)

    #     # Move the turtle to the target coordinates
    #     self.move_straight(distance_to_target)
    
    def rotate(self, angle:float, clockwise:bool = False, angular_speed:float = 1.0):
        """
        Rotate the turtle by the specified angle in the specified direction.
        """
        # Track the initial speed before stopping
        initial_speed = self.vel_msg
        
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

        # Restore the initial speed
        self.set_velocity(speed_twist_obj=initial_speed)
        # TODO: Fix: Is this above line even required?

    def move_straight(self, distance, axis='x'):
        """
        Move the turtle in a straight line for the specified distance.
        Axis can be 'x', 'y', or 'z' to indicate movement direction.
        """
        if axis not in ('x', 'y', 'z'):
            raise ValueError("Invalid axis. Choose from 'x', 'y', or 'z'.")

        # Set the linear speed based on the chosen axis
        linear_speed = self.get_axis_speed(axis)
        start_time = time.time()

        while time.time() - start_time < distance / abs(linear_speed) and not rospy.is_shutdown():
            # Set the velocity along the specified axis
            if axis == 'x':
                self.set_velocity(linear_x=linear_speed)
            elif axis == 'y':
                self.set_velocity(linear_y=linear_speed)
            elif axis == 'z':
                self.set_velocity(linear_z=linear_speed)

        # Stop after moving the specified distance
        self.stop()

    ## Getters
    def print_speeds(self):
        """
        Print the current linear and angular speeds of the turtle.
        """
        print(f"Linear speeds: {self.linear_speed}")
        print(f"Angular speeds: {self.angular_speed}")

# Example usage
# if __name__ == "__main__":
#     mover = TurtleMover(linear_speed=(1.0, 0.0, 0.0), angular_speed=(0.0, 0.0, 0.0))
#     mover.reset_turtlesim(0)
#     mover.move_straight(4)
#     mover.rotate(math.pi / 2, clockwise=False, angular_speed=1)
#     mover.move()
#     mover.move_for_duration(1)