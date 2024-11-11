# Tasks
1. Add a starting position i.e. coordinates of the turtle in the TurtleMover __init__ method.
2. Implement `get_to_coordinates()`
3. Fix: Remove the hardcoded set linear_x velocity to 1.0 from `TurtleMover::move_for_duration()`
4. `TurtleMover::rotate()` uses math.pi and it gives in accurate results. Use math.radians() instead.
5. Get current coordinates of the turtle
6. Project currently has unnecessary function arguments with default values for speed and easy testing. **Remove them**.
7. Add proper try-except blocks to handle exceptions or remove them if not needed.
8. Create documentation
9. Fix the images heights and widths in the README.md and center them.
10. See what's a better way of adding speeds as the command line arguments. Preferably, in format --linear 1.0 2.0 0 --angular 1.0 2.0 0 or (1.0, 2.0, 0) (1.0, 2.0, 0) or something similar.
11. Add logging in the `TurtleMover` class only either using the `logging` module or the `rospy.loginfo()` method. Either as a method or decorator or something else.
12. See how `Pose()` works
13. Currently the angles are in pi radians. See if there is a better way to get accurate rotations.
14. TOCHECK: Line 14 in `move_straight.py` is not needed. Remove it.
15. Add speeds as the command line arguments for the `move_straight.py` script.

# Notes
1. Currently `move_straight()` sets the linear velocity to 1.0 if it's zero in the axis of movement, while keeping the other existing speeds as well. Give better warnings or handle it better. Just remember that it might not always move on the axis.
2. I was getting the error saying that I am sending a different hash kind of than the expected one. This happened maybe because the method was expecting (x,y) but I was sending (x,y,z). I tried this for the `get_current_coordinates()` method.
3. `TurtleMover::move_straight()` seems like it will only move in stright line, but it also takes in angular_speed. Like, is it required, or should I make it independent of the angular speed?