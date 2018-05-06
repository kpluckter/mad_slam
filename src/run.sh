gnome-terminal -e roscore &
sleep 3;
rosbag play -d 1  good1.bag &
sleep 6;
python2 map_odom.py ../drone_data2/cmu_map.png 0.0323 64.5 51
rosrun 
