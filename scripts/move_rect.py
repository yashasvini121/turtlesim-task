#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from time import sleep

def move_straight(velocity_publisher, vel_msg, distance, speed):
    # Move forward
    vel_msg.linear.x = speed
    vel_msg.angular.z = 0.0
    start_time = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - start_time < distance / speed:
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.01)

    # Stop after moving the required distance
    vel_msg.linear.x = 0.0
    velocity_publisher.publish(vel_msg)
    sleep(1)  # Pause briefly before the next move

def turn(velocity_publisher, vel_msg, angle, angular_speed):
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
    sleep(1)  # Pause briefly before the next move

def move_turtle_rectangle(length, breadth, speed=1.0, angular_speed=1.0):
    # Initialize the ROS node
    rospy.init_node('move_turtle_rectangle_node', anonymous=True)
    
    # Create a publisher to send velocity commands to the turtle
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Create a Twist message
    vel_msg = Twist()

    rospy.loginfo("Moving the turtle in a rectangular path...")

    # Define the turning angle (90 degrees in radians)
    turn_angle = 1.57  # Approximately pi/2

    # Repeat the rectangle path 4 times (length, breadth, length, breadth)
    for _ in range(2):
        # Move along length
        move_straight(velocity_publisher, vel_msg, length, speed)
        
        # Turn 90 degrees
        turn(velocity_publisher, vel_msg, turn_angle, angular_speed)
        
        # Move along breadth
        move_straight(velocity_publisher, vel_msg, breadth, speed)
        
        # Turn 90 degrees
        turn(velocity_publisher, vel_msg, turn_angle, angular_speed)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: rosrun <package_name> <script_name> <length> <breadth>")
    else:
        try:
            # Get length and breadth from command line arguments
            length = float(sys.argv[1])
            breadth = float(sys.argv[2])
            
            # Call the function to move the turtle in a rectangular path
            move_turtle_rectangle(length, breadth)
        except rospy.ROSInterruptException:
            pass
