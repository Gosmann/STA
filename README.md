# STA

## useful commands

sudo dd bs=4M if=/dev/mmcblk0 status=progress of=/media/gosmann/external_Ext4/image3.img.gz

gzip -v image2-131023.img

### setup slam

gosmann@gosmann-G3-3579:~/ros2_ws$ ros2 run mobile_robot mobile_robot

gosmann@gosmann-G3-3579:~/ros2_ws$ ros2 launch slam_toolbox online_async_launch.py params_file:=./mapper_params_online_async.yaml

gosmann@gosmann-G3-3579:~/ros2_ws$ ros2 shark shark

gosmann@gosmann-G3-3579:~/ros2_ws$ rviz2

gosmann@gosmann-G3-3579:~/ros2_ws$ ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap

pi@raspberrypi:~$ ros2 launch ydlidar_ros2_driver ydlidar_launch.py

pi@raspberrypi:~$ ros2 launch ydlidar_ros2_driver ydlidar_launch_mod.py



