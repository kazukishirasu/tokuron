for i in `seq 1`
do
  roslaunch tsukuba turtlebot3_navigation.launch world_name:=/home/kazuki/tsukuba_ws/src/tsukuba/world/willow_garage.world map_file:=/home/kazuki/tsukuba_ws/src/tsukuba/map/willowgarage.yaml config_file:=/home/kazuki/tsukuba_ws/src/waypoint_manager/waypoint_server/config/waypoint_server.yaml x_pos:=0.0 y_pos:=0.0 z_pos:=0.15 yaw_pos:=0.0 initial_pose_x:=-11.0 initial_pose_y:=-16.6 initial_pose_a:=0.0 
done
