# About

This ROS Package can be used to control the movement of a bot inside the `turtlesim` package. It provides scripts to move the turtle in specific patterns, such as a straight line or a rectangle, with customizable parameters.

This repository is built upon [topguns837/ros_session](https://github.com/topguns837/ros_session).

# Scripts

## `move_rect.py`

This script moves the turtle in a rectangular pattern with specified length and breadth.

### Arguments

1. **length** (float) - The length of the rectangle.
2. **breadth** (float) - The breadth of the rectangle.

```bash
rosrun ros_session move_rect.py 3 2     # Example
```

This command makes the turtle move in a rectangle with a length of 3 units and a breadth of 2 units.

---

## `improved_rect.py`
This script moves the turtle in a rectangular pattern with specified length and breadth, the initial direction in radians, and the clockwise or anticlockwise direction.

### Arguments
1. **length** (float) - The length of the rectangle.
2. **breadth** (float) - The breadth of the rectangle.
3. **init_direction_radian** (float) - The initial direction of the turtle in radians.
4. **clockwise** (boolean) - True for clockwise, False for anticlockwise.

```bash
rosrun ros_session improved_rect.py 3 2 0 False     # Example
```

# Installation

- Install the `turtlesim` package:

    ```bash
    sudo apt-get install ros-noetic-turtlesim
    ```

- Fork the repository.
- Clone this package inside a catkin workspace:

    ```bash
    mkdir -p ros_ws/src 
    cd ros_ws/src
    git clone https://github.com/<your-username>/ros_session.git
    ```

- Compile the code and source the workspace:

    ```bash
    catkin_make
    source devel/setup.bash
    ```

# Instructions to Run the Code

- **Step 1**: Open a new terminal and start the ROS master node:

    ```bash
    roscore
    ```

- **Step 2**: Open another terminal and launch the `turtlesim` node:

    ```bash
    rosrun turtlesim turtlesim_node
    ```

- **Step 3**: In a third terminal, check that all scripts are executable by running the following command. Executable scripts will have an `x` in the permissions column.

    ```bash
    ls -l src/ros_session/scripts/
    ```

- **Step 4**: Make non-executable scripts executable by running:

    ```bash
    chmod +x src/ros_session/scripts/<script_name>
    ```

- **Step 5**: Run the desired script to control the turtle's movement.

---