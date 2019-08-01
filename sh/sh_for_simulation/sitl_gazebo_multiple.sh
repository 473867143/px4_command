##sitl_gazebo
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_command three_uav_mavros_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_command uav1.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_command uav2.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch px4_command uav3.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; rosrun px4_command multiple_move; exec bash"' \

