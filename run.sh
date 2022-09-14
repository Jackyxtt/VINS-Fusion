roslaunch vins vins_rviz.launch
rosrun vins vins_node /media/yxt/storage/SLAM_demo/VINS_FUSION_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml 
(optional) rosrun loop_fusion loop_fusion_node /media/yxt/storage/SLAM_demo/VINS_FUSION_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml 
rosbag play /media/yxt/storage/V1_02_medium.bag
