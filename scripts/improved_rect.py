#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from time import sleep

def move_straight(velocity_publisher, vel_msg, distance, speed):
    vel_msg.linear.x = speed
    vel_msg.angular.z = 0.0
    start_time = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - start_time < distance / speed:
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.01)

    vel_msg.linear.x = 0.0
    velocity_publisher.publish(vel_msg)
    sleep(1)

def turn(velocity_publisher, vel_msg, angle, angular_speed):
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = angular_speed
    start_time = rospy.Time.now().to_sec()

    while rospy.Time.now().to_sec() - start_time < angle / abs(angular_speed):
        velocity_publisher.publish(vel_msg)
        rospy.sleep(0.01)

    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)
    sleep(1)

def move_turtle_rectangle(origin_x, origin_y, length, breadth, direction, speed=1.0, angular_speed=1.0):
    rospy.init_node('move_turtle_rectangle_node', anonymous=True)
    
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Teleport the turtle to the starting origin point
    rospy.wait_for_service('/turtle1/teleport_absolute')
    teleport_service = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
    teleport_service(origin_x, origin_y, 0)  # Set initial heading

    vel_msg = Twist()
    rospy.loginfo("Moving the turtle in a rectangular path...")

    turn_angle = 1.57  # 90 degrees in radians

    # Define movement pattern based on the direction
    if direction == 'n':
        # Moves: right, up, left, down
        sides = [(length, speed), (breadth, speed), (length, -speed), (breadth, -speed)]
    elif direction == 's':
        # Moves: right, down, left, up
        sides = [(length, speed), (breadth, -speed), (length, -speed), (breadth, speed)]
    elif direction == 'e':
        # Moves: down, right, up, left
        sides = [(breadth, -speed), (length, speed), (breadth, speed), (length, -speed)]
    elif direction == 'w':
        # Moves: up, left, down, right
        sides = [(breadth, speed), (length, -speed), (breadth, -speed), (length, speed)]
    else:
        rospy.logwarn("Invalid direction specified. Use 'n' for North, 's' for South, 'e' for East, or 'w' for West.")
        return

    for side_length, move_speed in sides:
        # Move along the side
        move_straight(velocity_publisher, vel_msg, abs(side_length), abs(move_speed))
        # Turn 90 degrees to the right for the next side
        turn(velocity_publisher, vel_msg, turn_angle, angular_speed)

if __name__ == '__main__':
    if len(sys.argv) != 5:
        print("Usage: rosrun <package_name> <script_name> <origin_x,origin_y> <length> <breadth> <direction>")
    else:
        try:
            # Parse the origin coordinates
            origin = sys.argv[1].split(',')
            origin_x = float(origin[0])
            origin_y = float(origin[1])
            length = float(sys.argv[2])
            breadth = float(sys.argv[3])
            direction = sys.argv[4].lower()

            move_turtle_rectangle(origin_x, origin_y, length, breadth, direction)
        except rospy.ROSInterruptException:
            pass
