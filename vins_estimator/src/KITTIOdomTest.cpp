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

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "estimator/estimator.h"
#include "utility/visualization.h"

using namespace std;
using namespace Eigen;

Estimator estimator;

Eigen::Matrix3d c1Rc0, c0Rc1;
Eigen::Vector3d c1Tc0, c0Tc1;


void printVector(vector<double>& v )
{
	for (vector<double>::iterator it=v.begin(); it!=v.end(); it++)
	{
		std::cout<<*it<<" ";
	}
	std::cout<<std::endl;
}


void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<Vector3d> &vAcc, vector<Vector3d> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(" ")) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            // item = s.substr(0, pos);
            // data[6] = stod(item);

            vTimeStamps.push_back(data[0]);
            vAcc.push_back(Vector3d(data[1],data[2],data[3]));
            vGyro.push_back(Vector3d(data[4],data[5],data[6]));
			// std::cout<<"vTimeStamps= "<<vTimeStamps.back()<<std::endl;
			// std::cout<<"vAcc= "<<vAcc.back()<<std::endl;
			// std::cout<<"vGyro= "<<vGyro.back()<<std::endl;
			//estimator.inputIMU(t.back(), acc.back(), gyr.back());
        }
    }
	//std::cout<<vTimeStamps.size()<<std::endl;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "vins_estimator");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

	ros::Publisher pubLeftImage = n.advertise<sensor_msgs::Image>("/leftImage",1000);
	ros::Publisher pubRightImage = n.advertise<sensor_msgs::Image>("/rightImage",1000);

	if(argc != 3)
	{
		printf("please intput: rosrun vins kitti_odom_test [config file] [data folder] \n"
			   "for example: rosrun vins kitti_odom_test "
			   "~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml "
			   "/media/yxt/storage/2011_10_03_drive_0042_sync \n");
		return 1;
	}

	string config_file = argv[1];
	printf("config_file: %s\n", argv[1]);
	string sequence = argv[2];
	printf("read sequence: %s\n", argv[2]);
	string dataPath = sequence + "/";

	readParameters(config_file);
	estimator.setParameter();
	registerPub(n);
	//std::cout<<"finishing setting parameters"<<std::endl;
	// load image list
	FILE* file;
	file = std::fopen((dataPath + "/image_00/timestamps.txt").c_str() , "r");
	if(file == NULL){
	    printf("cannot find file: %stimes.txt\n", dataPath.c_str());
	    ROS_BREAK();
	    return 0;          
	}
	//std::cout<<"finishing openning target file"<<std::endl;
	// double imageTime;
	// vector<double> imageTimeList;
	// while ( fscanf(file, "%lf", &imageTime) != EOF)
	// {
	//     imageTimeList.push_back(imageTime);
	// }
	// std::fclose(file);
	// std::cout<<imageTimeList.size()<<std::endl;

	vector<double> imageTimeList;
	int year, month, day;
	int hour, minute;
	double second;
	while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF)
	{
		//printf("%lf\n", second);
	    imageTimeList.push_back(hour * 60 * 60 + minute * 60 + second);
	}
	std::fclose(file);
	std::cout<<imageTimeList.size()<<std::endl;

	string leftImagePath, rightImagePath;
	cv::Mat imLeft, imRight;
	FILE* outFile;
	outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(),"w");
	if(outFile == NULL)
		printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());


	vector<Vector3d> vAcc, vGyro;
	vector<double> vTimestampsImu;
	string pathImu = dataPath + "/imu_data_100hz/imu.txt";
	LoadIMU(pathImu, vTimestampsImu, vAcc, vGyro);
	// std::cout<<"finishing loading IMU"<<std::endl;
	printVector(vTimestampsImu);


	float diff=imageTimeList[0]-vTimestampsImu[0];

	int ii=0;
	for (size_t i = 0; i < imageTimeList.size(); i++)
	{	
		if(ros::ok())
		{
			printf("\nprocess image %d\n", (int)i);
			stringstream ss;
			ss << setfill('0') << setw(10) << i;
			leftImagePath = dataPath + "image_00/data/" + ss.str() + ".png";
			rightImagePath = dataPath + "image_01/data/" + ss.str() + ".png";
			// std::cout<<leftImagePath<<std::endl;
			//printf("%lu  %f \n", i, imageTimeList[i]);
			//printf("%s\n", leftImagePath.c_str() );
			//printf("%s\n", rightImagePath.c_str() );

			imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE );
			// std::cout<<"finishing reading image reading"<<std::endl;
			
			sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
			// std::cout<<"finishing converting into ROSMsg"<<std::endl;
			
			imLeftMsg->header.stamp = ros::Time(imageTimeList[i]);
			pubLeftImage.publish(imLeftMsg);
			// std::cout<<"finishing publishing ROSMsg"<<std::endl;

			imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE );
			sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
			imRightMsg->header.stamp = ros::Time(imageTimeList[i]);
			pubRightImage.publish(imRightMsg);

			while(vTimestampsImu[ii]+diff<=imageTimeList[i])
			{
				std::cout<<"imu_time="<<vTimestampsImu[ii]+diff-0.5<<" "<<"image_time="<<imageTimeList[i]<<" ii="<<ii<<std::endl;
				
				estimator.inputIMU(vTimestampsImu[ii], vAcc[ii], vGyro[ii]);
				ii++;
			}
			estimator.inputImage(imageTimeList[i], imLeft, imRight);
			



			Eigen::Matrix<double, 4, 4> pose;
			estimator.getPoseInWorldFrame(pose);
			if(outFile != NULL)
				fprintf (outFile, "%f %f %f %f %f %f %f %f %f %f %f %f \n",pose(0,0), pose(0,1), pose(0,2),pose(0,3),
																	       pose(1,0), pose(1,1), pose(1,2),pose(1,3),
																	       pose(2,0), pose(2,1), pose(2,2),pose(2,3));
			
			//cv::imshow("leftImage", imLeft);
			//cv::imshow("rightImage", imRight);
			//cv::waitKey(2);
		}
		else
			break;
	}
	if(outFile != NULL)
		fclose (outFile);
	return 0;
}
