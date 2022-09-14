roslaunch vins vins_rviz.launch
rosrun vins kitti_gps_test /media/yxt/storage/SLAM_demo/VINS_FUSION_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml /media/yxt/storage/2011_10_03_drive_0042_sync
rosrun global_fusion global_fusion_node