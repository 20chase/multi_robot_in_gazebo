# multi_robot_in_gazebo
Create multiple turtlebots in the Gazebo simulator

## Environment
- Ubuntu 16.04
- ROS Kinect
- Gazebo 7.0+ (Recommand Gazebo 9.0 which support parallel physics)

## How to use it
Put these packages into your ROS Workspace (${WS}/src), and

    catkin_make
    
Then, start multi-robot simulation

    roslaunch turtlebot_gazebo test.launch world_file:=${YOUR_WORLD_FILE}
    
[Option] start gazebo gui

    gzclient
    
## How to create different multi-robot scenarios
Go to launch file

    cd turtlebot_hokuyo/turtlebot_gazebo/launch
    
Create a scenario which have ten robots

    python create_multi_robots.py --robot_num 10 --env_size 8
    
Finally, the new test.launch file would be generated!




