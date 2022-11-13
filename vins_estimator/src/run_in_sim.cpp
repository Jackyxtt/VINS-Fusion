
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "../include/System.h"
#include "utility/tic_toc.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/dataset/EuRoC/MH-05/mav0/";
string sConfig_path = "../config/";

std::shared_ptr<System> pSystem;

void PubImuData()
{
//	string sImu_data_file = sConfig_path + "MH_05_imu0.txt";
    string sImu_data_file = "/media/yxt/storage/github_useful_tools/vio_data_simulation/bin/imu_pose.txt";
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
        ssImuData >> dStampNSec;//时间戳
        for(int i=0;i<7;i++) //利用循环跳过imu quaternion(4)和imu position(3)
            ssImuData>>tmp;
		ssImuData >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec, vGyr, vAcc);
//		usleep(2500*nDelayTimes);//5ms, 200hz
	}
	fsImu.close();
}

void PubSimImageData(){
    string sImage_file = "/media/yxt/storage/github_useful_tools/vio_data_simulation/bin/cam_pose.txt";

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
//    string sImgFileName;
    int n = 0;

    while (std::getline(fsImage, sImage_line) && !sImage_line.empty()){
        std::istringstream ssImuData(sImage_line);
        ssImuData >> dStampNSec;
//        cout<<"cam time: "<<fixed<<dStampNSec<<endl;
        //cam_pose.txt中相机数与keyframe中每一帧一一对应，从all_points_0.txt到all_points_600
        string imagePath = "/media/yxt/storage/github_useful_tools/vio_data_simulation/bin/keyframe/all_points_"
                + std::to_string(n) + ".txt";
//        cout<<"points_file: "<<imagePath<<endl;
        vector<pair<int, cv::Point2f>> FeaturePoints;//容器FeaturePoints存放一个相机的特征点(归一化坐标)
        ifstream f;
        f.open(imagePath.c_str());

        if (!f.is_open())
        {
            cerr << "Failed to open image file! " << imagePath << endl;
            return;
        }

        std::string s;
        while (std::getline(f, s) && !s.empty()){
            std::istringstream ss(s);
            double tmp;
            int n_id;
            ss >> n_id;
            for (int i = 0; i < 4; i++)
                ss >> tmp;
            float px, py;
            ss >> px;
            ss >> py;
            cv::Point2f pt(px, py);
            FeaturePoints.push_back(make_pair(n_id,pt));

        }
        pSystem->PubSimImageData(FeaturePoints, dStampNSec);
//        usleep(50000*nDelayTimes);//usleep延时时间单位为微秒，百万分之一,100ms ,10Hz
        n++;
    }
    fsImage.close();

}
#ifdef __APPLE__
// support for MacOS
void DrawIMGandGLinMainThrd(){
	string sImage_file = sConfig_path + "MH_05_cam0.txt";

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

	pSystem->InitDrawGL();
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
		//pSystem->PubImageData(dStampNSec / 1e9, img);
		cv::Mat show_img;
		cv::cvtColor(img, show_img, CV_GRAY2RGB);
		if (SHOW_TRACK)
		{
			for (unsigned int j = 0; j < pSystem->trackerData[0].cur_pts.size(); j++)
			{
				double len = min(1.0, 1.0 *  pSystem->trackerData[0].track_cnt[j] / WINDOW_SIZE);
				cv::circle(show_img,  pSystem->trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
			}

			cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
			cv::imshow("IMAGE", show_img);
		  // cv::waitKey(1);
		}

		pSystem->DrawGLFrame();
		usleep(50000*nDelayTimes);
	}
	fsImage.close();

} 
#endif

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

	pSystem.reset(new System(sConfig_path));

    PubImuData();
    PubSimImageData();
	
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);

//	std::thread thd_PubImuData(PubImuData);

//    std::thread thd_PubImageData(PubSimImageData);

#ifdef __linux__	
	std::thread thd_Draw(&System::Draw, pSystem);
#elif __APPLE__
	DrawIMGandGLinMainThrd();
#endif

//	thd_PubImuData.join();
//	thd_PubImageData.join();

	 thd_BackEnd.join();
	// thd_Draw.join();

	cout << "main end... see you ..." << endl;
	return 0;
}
