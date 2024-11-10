#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from time import sleep
from math import pi

def move_straight(velocity_publisher, vel_msg, distance, speed):
    """
    Function to move the turtle in a straight line at a given speed for a specified distance.
    Then pause briefly for 1 second before the next move.
    """
    vel_msg.linear.x = speed
    vel_msg.angular.z = 0.0
    start_time = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - start_time < distance / speed:
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.01)

    # Stop after moving the required distance
    vel_msg.linear.x = 0.0
    velocity_publisher.publish(vel_msg)
    sleep(1)

def turn(velocity_publisher, vel_msg, angle, angular_speed):
    """
    Function to turn the turtle by a specified angle at a given angular speed.
    Then pause briefly for 1 second before the next move.
    """

    # Turn with given angular speed
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = angular_speed
    start_time = rospy.Time.now().to_sec()

    # Time to turn 90 degrees (1.57 radians) = angle / angular_speed
    while rospy.Time.now().to_sec() - start_time < angle / abs(angular_speed):
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.01)

    # Stop turning
    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)
    sleep(1)

def move_turtle_rectangle(length, breadth, speed=1.0, angular_speed=1.0):
    """
    Function to move the turtle in a rectangular path with the given length and breadth.
    The turtle moves forward by the specified length, turns 90 degrees, moves forward by the specified breadth, turns 90 degrees, and repeats this process to complete the rectangular path.
    """
    # Initialize the ROS node
    rospy.init_node('move_turtle_rectangle_node', anonymous=True)
    
    # Create a publisher to send velocity commands to the turtle
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Create a Twist message
    vel_msg = Twist()

    rospy.loginfo("Moving the turtle in a rectangular path...")

    # Define the turning angle (90 degrees in radians)
    turn_angle = pi / 2

    # Repeat the rectangle path 4 times (length, breadth, length, breadth)
    for _ in range(2):
        move_straight(velocity_publisher, vel_msg, length, speed)
        turn(velocity_publisher, vel_msg, turn_angle, angular_speed)
        move_straight(velocity_publisher, vel_msg, breadth, speed)
        turn(velocity_publisher, vel_msg, turn_angle, angular_speed)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: rosrun <package_name> <script_name> <length> <breadth>")
    else:
        try:
            # User Input
            length = float(sys.argv[1])
            breadth = float(sys.argv[2])
            
            # move_turtle_rectangle Function Call
            move_turtle_rectangle(length, breadth)
        except rospy.ROSInterruptException:
            pass
