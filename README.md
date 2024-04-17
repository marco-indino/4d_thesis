To run the simulation:

`ros2 launch scout_2 gps_gazebo.launch.py use_sim_time:=true`

Then you need to localize the robot:

`ros2 launch scout_2 dual_ekf_navsat.launch.py use_sim_time:=true`

Then you can start the navigation:

`ros2 launch scout_2 dual_ekf_navsat.launch.py use_sim_time:=true`

If you want to save a trajectory you need to run the command below and then move the robot within the environment with the teleop_key:

`ros2 run save_poses save_poses_node`

If you want the robot to follow the trajectory you have recorded you need to run the command ( having already started simulation, localization and navigation):

`ros2 run nav_t_poses nav_t_poses_exe`
