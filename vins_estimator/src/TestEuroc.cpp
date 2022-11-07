/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "../include/estimator.h"
#include "../include/parameters.h"
#include "../include/System.h"
#include <highgui.h>

using namespace std;
using namespace cv;
using namespace Eigen;
// estimator 应该出现在System.cpp中
//Estimator estimator;
string sData_path = "/home/dataset/EuRoC/MH-05/mav0/";
string sConfig_path = "../config/";

std::shared_ptr<System> pSystem;
//queue<sensor_msgs::ImuConstPtr> imu_buf;
//queue<sensor_msgs::PointCloudConstPtr> feature_buf;
// queue<sensor_msgs::ImageConstPtr> img0_buf;
//queue<sensor_msgs::CompressedImageConstPtr> img0_buf; //压缩图像
// queue<sensor_msgs::ImageConstPtr> img1_buf;
//queue<sensor_msgs::CompressedImageConstPtr> img1_buf; //压缩图像
//std::mutex m_buf;

//void img0_callback(const sensor_msgs::CompressedImageConstPtr &img_msg)
//{
//    m_buf.lock();
//    img0_buf.push(img_msg);
//    m_buf.unlock();
//}
//
//void img1_callback(const sensor_msgs::CompressedImageConstPtr &img_msg)
//{
//    m_buf.lock();
//    img1_buf.push(img_msg);
//    m_buf.unlock();
//}

void PubImuData()
{
    string sImu_data_file = "./config/MH_05_imu0.txt";
//    string sImu_data_file = "/media/yxt/storage/github_useful_tools/vio_data_simulation/bin/imu_pose.txt";
    cout << "1 PubImuData start sImu_data_file: " << sImu_data_file << endl;
    ifstream fsImu;
    fsImu.open(sImu_data_file.c_str());
    if (!fsImu.is_open())
    {
        cerr << "Failed to open imu file! " << sImu_data_file << endl;
        return;
    }

    std::string sImu_line;
    double dStampNSec = 0.0;
    Vector3d vAcc;
    Vector3d vGyr;
    double tmp;
    while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
    {
        std::istringstream ssImuData(sImu_line);
        ssImuData >> dStampNSec >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
        // cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
        pSystem->PubImuData(dStampNSec / 1e9, vGyr, vAcc);
//		usleep(5000*nDelayTimes);//10000um = 0.01s 100hz
    }
    fsImu.close();
}

void PubImageData()
{
    string sImage_file = "./config/MH_05_cam0.txt";

    cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

    ifstream fsImage;
    fsImage.open(sImage_file.c_str());
    if (!fsImage.is_open())
    {
        cerr << "Failed to open image file! " << sImage_file << endl;
        return;
    }

    std::string sImage_line;
    double dStampNSec;
    string sImgFileName;

    // cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
    while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
    {
        std::istringstream ssImuData(sImage_line);
        ssImuData >> dStampNSec >> sImgFileName;
        // cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
        string imagePath = sData_path + "cam0/data/" + sImgFileName;

        Mat img = imread(imagePath.c_str(), 0);
        if (img.empty())
        {
            cerr << "image is empty! path: " << imagePath << endl;
            return;
        }
        pSystem->PubImage0Data(dStampNSec / 1e9, img);
        pSystem->PubImage1Data(dStampNSec / 1e9, img);
        // cv::imshow("SOURCE IMAGE", img);
        // cv::waitKey(0);
//		usleep(50000*nDelayTimes); //100ms 10Hz
    }
    fsImage.close();
}


//void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
//{
//    double t = imu_msg->header.stamp.toSec();
//    double dx = imu_msg->linear_acceleration.x;
//    double dy = imu_msg->linear_acceleration.y;
//    double dz = imu_msg->linear_acceleration.z;
//    double rx = imu_msg->angular_velocity.x;
//    double ry = imu_msg->angular_velocity.y;
//    double rz = imu_msg->angular_velocity.z;
//    Vector3d acc(dx, dy, dz);
//    Vector3d gyr(rx, ry, rz);
//    estimator.inputIMU(t, acc, gyr);
//    return;
//}


//void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
//{
//    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
//    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
//    {
//        int feature_id = feature_msg->channels[0].values[i];
//        int camera_id = feature_msg->channels[1].values[i];
//        double x = feature_msg->points[i].x;
//        double y = feature_msg->points[i].y;
//        double z = feature_msg->points[i].z;
//        double p_u = feature_msg->channels[2].values[i];
//        double p_v = feature_msg->channels[3].values[i];
//        double velocity_x = feature_msg->channels[4].values[i];
//        double velocity_y = feature_msg->channels[5].values[i];
//        if(feature_msg->channels.size() > 5)
//        {
//            double gx = feature_msg->channels[6].values[i];
//            double gy = feature_msg->channels[7].values[i];
//            double gz = feature_msg->channels[8].values[i];
//            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
//            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
//        }
//        ROS_ASSERT(z == 1);
//        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
//        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
//        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
//    }
//    double t = feature_msg->header.stamp.toSec();
//    estimator.inputFeature(t, featureFrame);
//    return;
//}

//void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
//{
//    if (restart_msg->data == true)
//    {
//        printf("restart the estimator!");
//        estimator.clearState();
//        estimator.setParameter();
//    }
//    return;
//}

//void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
//{
//    if (switch_msg->data == true)
//    {
//        //printf("use IMU!");
//        estimator.changeSensorType(1, STEREO);
//    }
//    else
//    {
//        //printf("disable IMU!");
//        estimator.changeSensorType(0, STEREO);
//    }
//    return;
//}

//void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
//{
//    if (switch_msg->data == true)
//    {
//        //printf("use stereo!");
//        estimator.changeSensorType(USE_IMU, 1);
//    }
//    else
//    {
//        //printf("use mono camera_models (left)!");
//        estimator.changeSensorType(USE_IMU, 0);
//    }
//    return;
//}

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config/file.yaml \n"
             << "For example: ./run_euroc /home/stevencui/dataset/EuRoC/MH-05/mav0/ ../config/file.yaml"<< endl;
        return -1;
    }
    sData_path = argv[1];//第一个参数随便写的不重要
    sConfig_path = argv[2];
    printf("config_file: %s\n", sConfig_path);
    pSystem.reset(new System(sConfig_path));
//    readParameters(sConfig_path);
//    estimator.setParameter();

    PubImuData();
    PubImageData();

#ifdef EIGEN_DONT_PARALLELIZE
    printf("EIGEN_DONT_PARALLELIZE");
#endif



//    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
//    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);//此话题并未被发送
//    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
//    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
//    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
//    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
//    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);
      std::thread thd_Draw(&System::Draw, pSystem);
      pSystem->sync_process();
      while(1){
      }
//    ros::spin();

    return 0;
}
