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

void PubImuData()
{
    string sImu_data_file = "./config/euroc/MH_05_imu0.txt";
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
    string sImage_file = "./config/euroc/MH_05_cam0.txt";

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
        string image0Path = sData_path + "cam0/data/" + sImgFileName;
        string image1Path = sData_path + "cam1/data/" + sImgFileName;

        Mat img0 = imread(image0Path.c_str(), 0);
        Mat img1 = imread(image1Path.c_str(), 0);
        if (img0.empty())
        {
            cerr << "image0 is empty! path: " << image0Path << endl;
            return;
        }

        if (img1.empty())
        {
            cerr << "image1 is empty! path: " << image0Path << endl;
            return;
        }

        pSystem->PubImage0Data(dStampNSec / 1e9, img0);
        pSystem->PubImage1Data(dStampNSec / 1e9, img1);
//         cv::imshow("SOURCE IMAGE", img1);
//         cv::waitKey(2);
//		usleep(50000*nDelayTimes); //100ms 10Hz
    }
    fsImage.close();
}

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
    cout <<"config_file: " << sConfig_path << endl;
    pSystem.reset(new System(sConfig_path));
//    readParameters(sConfig_path);
//    estimator.setParameter();

    PubImuData();
    PubImageData();
//    pSystem->pubIMUtoEstimator();
//    pSystem->sync_process();
//    std::thread thd_PubImuData(&PubImuData);
//    std::thread thd_PubImageData(&PubImageData);
    std::thread thd_pubIMUtoEstimator(&System::pubIMUtoEstimator, pSystem);
    std::thread thd_sync_process(&System::sync_process, pSystem);

#ifdef EIGEN_DONT_PARALLELIZE
    printf("EIGEN_DONT_PARALLELIZE\n");
#endif

      std::thread thd_Draw(&System::Draw, pSystem);
      pSystem->estimator.createProcessthd();
//      std::thread thd_process(&Estimator::processMeasurements, pSystem->estimator);
      while(1){
      }
//    ros::spin();

    return 0;
}
