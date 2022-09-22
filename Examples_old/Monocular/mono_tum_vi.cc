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
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"
#include "Converter.h"



void LoadImages(const std::string &strImagePath, const std::string &strPathTimes,
                std::vector<std::string> &vstrImages, std::vector<double> &vTimeStamps);

double ttrack_tot = 0;
int main(int argc, char **argv)
{
    const int num_seq = (argc-3)/2;
    std::cout << "num_seq = " << num_seq << std::endl;
    bool bFileName= (((argc-3) % 2) == 1);

    std::string file_name;
    if (bFileName)
    {
        file_name = std::string(argv[argc-1]);
        std::cout << "file name: " << file_name << std::endl;
    }


    if(argc < 4)
    {
        std::cerr << std::endl << "Usage: ./mono_tum_vi path_to_vocabulary path_to_settings path_to_image_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << std::endl;
        return 1;
    }

    // Load all sequences:
    int seq;
    std::vector< std::vector<std::string> > vstrImageFilenames;
    std::vector< std::vector<double> > vTimestampsCam;
    std::vector<int> nImages;

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        std::cout << "Loading images for sequence " << seq << "...";
        LoadImages(std::string(argv[(2*seq)+3]), std::string(argv[(2*seq)+4]), vstrImageFilenames[seq], vTimestampsCam[seq]);
        std::cout << "LOADED!" << std::endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];

        if((nImages[seq]<=0))
        {
            std::cerr << "ERROR: Failed to load images for sequence" << seq << std::endl;
            return 1;
        }

    }
    // Vector for tracking time statistics
    std::vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    std::cout << std::endl << "-------" << std::endl;
    std::cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    int proccIm = 0;
    for (seq = 0; seq<num_seq; seq++)
    {

        // Main loop
        cv::Mat im;
        proccIm = 0;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {

            // Read image from file
            im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_GRAYSCALE); //,cv::IMREAD_GRAYSCALE);

            if(imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
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
            clahe->apply(im,im);


            // std::cout << "mat type: " << im.type() << std::endl;
            double tframe = vTimestampsCam[seq][ni];

            if(im.empty())
            {
                std::cerr << std::endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << std::endl;
                return 1;
            }
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

            // Pass the image to the SLAM system
            SLAM.TrackMonocular(im,tframe); // TODO change to monocular_inertial

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

    // std::cout << "ttrack_tot = " << ttrack_tot << std::endl;
    // Stop all threads
    SLAM.Shutdown();


    // Tracking time statistics

    // Save camera trajectory

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


void LoadImages(const std::string &strImagePath, const std::string &strPathTimes,
                std::vector<std::string> &vstrImages, std::vector<double> &vTimeStamps)
{
    std::ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        std::string s;
        getline(fTimes,s);

        if(!s.empty())
        {
            if (s[0] == '#')
                continue;

            int pos = s.find(' ');
            std::string item = s.substr(0, pos);

            vstrImages.push_back(strImagePath + "/" + item + ".png");
            double t = stod(item);
            vTimeStamps.push_back(t/1e9);
        }
    }
}


