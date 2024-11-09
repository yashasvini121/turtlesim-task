# About

This ROS Package can be used to control the movement of a bot inside the turtlesim package

# Installation

- Install the `turtlesim` package

    ```bash
    sudo apt-get install ros-noetic-turtlesim
    ```

- Fork the repository.
- Clone this package inside a catkin workspace

    ```bash
    mkdir -p ros_ws/src 
    cd ros_ws/src
    git clone https://github.com/<your-username>/ros_session.git
    ```

- Compile the code and source the workspace

    ```bash
    catkin_make; source devel/setup.bash
    ```

# Instructions to run the code

- In terminal 1

    ```bash
    roscore
    ```

- In terminal 2

    ```bash
    rosrun turtlesim turtlesim_node
    ```

- In terminal 3
    
    - Check that all the scripts are executable by running the following command:
    The executable scripts have a `x` in the permissions column

        ```bash
        ls -l src/ros_session/scripts/
        ```

    - Make the non executable scripts executable by running the following command
    
        ```bash
        chmod +x src/ros_session/scripts/<script_name>
        ```
    
    - Run the script to move the bot in a rectangle args: length, breadth

        ```bash
        rosrun ros_session <script_name> <optional args>
        ```
        ```
        rosrun ros_session move_rect.py 3 2
        ```
