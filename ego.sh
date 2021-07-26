## ego自主飞行脚本
gnome-terminal --window -e 'bash -c "roslaunch prometheus_gazebo sitl_ego_map4.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch prometheus_gazebo sitl_ego_px4.launch; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch prometheus_gazebo sitl_ego_planner.launch; exec bash"' \
--tab -e 'bash -c "sleep 20; roslaunch prometheus_gazebo sitl_ego_station.launch; exec bash"' \

