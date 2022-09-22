/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

void LoadImagesTUMVI(const std::string &strPathLeft, const std::string &strPathRight, const std::string &strPathTimes,
                std::vector<std::string> &vstrImageLeft, std::vector<std::string> &vstrImageRight, std::vector<double> &vTimeStamps);

void LoadIMU(const std::string &strImuPath, std::vector<double> &vTimeStamps, std::vector<cv::Point3f> &vAcc, std::vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;
int main(int argc, char **argv)
{
    const int num_seq = (argc-3)/4;
    std::cout << "num_seq = " << num_seq << std::endl;
    bool bFileName= (((argc-3) % 4) == 1);
    std::string file_name;
    if (bFileName)
        file_name = std::string(argv[argc-1]);

    if(argc < 7)
    {
        std::cerr << std::endl << "Usage: ./stereo_inertial_tum_vi path_to_vocabulary path_to_settings path_to_image_folder_1 path_to_image_folder_2 path_to_times_file path_to_imu_data (trajectory_file_name)" << std::endl;
        return 1;
    }


    // Load all sequences:
    int seq;
    std::vector< std::vector<std::string> > vstrImageLeftFilenames;
    std::vector< std::vector<std::string> > vstrImageRightFilenames;
    std::vector< std::vector<double> > vTimestampsCam;
    std::vector< std::vector<cv::Point3f> > vAcc, vGyro;
    std::vector< std::vector<double> > vTimestampsImu;
    std::vector<int> nImages;
    std::vector<int> nImu;
    std::vector<int> first_imu(num_seq,0);

    vstrImageLeftFilenames.resize(num_seq);
    vstrImageRightFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        std::cout << "Loading images for sequence " << seq << "...";
        LoadImagesTUMVI(std::string(argv[4*(seq+1)-1]), std::string(argv[4*(seq+1)]), std::string(argv[4*(seq+1)+1]), vstrImageLeftFilenames[seq], vstrImageRightFilenames[seq], vTimestampsCam[seq]);
        std::cout << "Total images: " << vstrImageLeftFilenames[seq].size() << std::endl;
        std::cout << "Total cam ts: " << vTimestampsCam[seq].size() << std::endl;
        std::cout << "first cam ts: " << vTimestampsCam[seq][0] << std::endl;

        std::cout << "LOADED!" << std::endl;

        std::cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(std::string(argv[4*(seq+1)+2]), vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        std::cout << "Total IMU meas: " << vTimestampsImu[seq].size() << std::endl;
        std::cout << "first IMU ts: " << vTimestampsImu[seq][0] << std::endl;
        std::cout << "LOADED!" << std::endl;

        nImages[seq] = vstrImageLeftFilenames[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            std::cerr << "ERROR: Failed to load images or IMU for sequence" << seq << std::endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first

        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered

    }

    // Vector for tracking time statistics
    std::vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    std::cout << std::endl << "-------" << std::endl;
    std::cout.precision(17);

    /*std::cout << "Start processing sequence ..." << std::endl;
    std::cout << "Images in the sequence: " << nImages << std::endl;
    std::cout << "IMU data in the sequence: " << nImu << std::endl << std::endl;*/

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    int proccIm = 0;
    for (seq = 0; seq<num_seq; seq++)
    {

        // Main loop
        cv::Mat imLeft, imRight;
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        proccIm = 0;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {

            // Read image from file
            imLeft = cv::imread(vstrImageLeftFilenames[seq][ni],cv::IMREAD_GRAYSCALE);
            imRight = cv::imread(vstrImageRightFilenames[seq][ni],cv::IMREAD_GRAYSCALE);

            if(imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
                int width = imLeft.cols * imageScale;
                int height = imLeft.rows * imageScale;
                cv::resize(imLeft, imLeft, cv::Size(width, height));
                cv::resize(imRight, imRight, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
                t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif
            }

            // clahe
            clahe->apply(imLeft,imLeft);
            clahe->apply(imRight,imRight);

            double tframe = vTimestampsCam[seq][ni];

            if(imLeft.empty() || imRight.empty())
            {
                std::cerr << std::endl << "Failed to load image at: "
                     <<  vstrImageLeftFilenames[seq][ni] << std::endl;
                return 1;
            }


            // Load imu measurements from previous frame
            vImuMeas.clear();

            if(ni>0)
            {
                // std::cout << "t_cam " << tframe << std::endl;

                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])
                {
                    // vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[first_imu],vGyro[first_imu],vTimestampsImu[first_imu]));
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    // std::cout << "t_imu = " << fixed << vImuMeas.back().t << std::endl;
                    first_imu[seq]++;
                }
            }

            /*std::cout << "first imu: " << first_imu[seq] << std::endl;
            std::cout << "first imu time: " << fixed << vTimestampsImu[seq][0] << std::endl;
            std::cout << "size vImu: " << vImuMeas.size() << std::endl;*/

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            SLAM.TrackStereo(imLeft,imRight,tframe,vImuMeas);

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            ttrack_tot += ttrack;
            // std::cout << "ttrack: " << ttrack << std::endl;

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                std::this_thread::sleep_for(std::chrono::microseconds(int((T-ttrack)*1e6)));
        }
        if(seq < num_seq - 1)
        {
            std::cout << "Changing the dataset" << std::endl;

            SLAM.ChangeDataset();
        }
    }


    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics

    // Save camera trajectory
    std::chrono::system_clock::time_point scNow = std::chrono::system_clock::now();
    std::time_t now = std::chrono::system_clock::to_time_t(scNow);
    std::stringstream ss;
    ss << now;

    if (bFileName)
    {
        const std::string kf_file =  "kf_" + std::string(argv[argc-1]) + ".txt";
        const std::string f_file =  "f_" + std::string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages[0]; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    std::cout << "-------" << std::endl << std::endl;
    std::cout << "median tracking time: " << vTimesTrack[nImages[0]/2] << std::endl;
    std::cout << "mean tracking time: " << totaltime/proccIm << std::endl;

    return 0;
}

void LoadImagesTUMVI(const std::string &strPathLeft, const std::string &strPathRight, const std::string &strPathTimes,
                std::vector<std::string> &vstrImageLeft, std::vector<std::string> &vstrImageRight, std::vector<double> &vTimeStamps)
{
    std::ifstream fTimes;
    std::cout << strPathLeft << std::endl;
    std::cout << strPathRight << std::endl;
    std::cout << strPathTimes << std::endl;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        std::string s;
        std::getline(fTimes,s);

        if(!s.empty())
        {
            if (s[0] == '#')
                continue;

            int pos = s.find(' ');
            std::string item = s.substr(0, pos);

            vstrImageLeft.push_back(strPathLeft + "/" + item + ".png");
            vstrImageRight.push_back(strPathRight + "/" + item + ".png");

            double t = stod(item);
            vTimeStamps.push_back(t/1e9);
        }
    }
}

void LoadIMU(const std::string &strImuPath, std::vector<double> &vTimeStamps, std::vector<cv::Point3f> &vAcc, std::vector<cv::Point3f> &vGyro)
{
    std::ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        std::string s;
        std::getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            std::string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != std::string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
