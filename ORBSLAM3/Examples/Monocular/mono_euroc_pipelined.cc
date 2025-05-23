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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <thread>

#include <opencv2/core/core.hpp>

#include <System.h>
#include "utils/ThreadSafeQueue.h"
#include "utils/FeatureExtractorTypes.h"
#include "PipelinedFE/PipelinedProcessFactory.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{  
    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_euroc_pipelined path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (output_trajectory_file_dir)" << endl;
        return 1;
    }

    const int num_seq = (argc-3)/2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageFilenames;
    vector< vector<double> > vTimestampsCam;
    vector<int> nImages;

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
        LoadImages(string(argv[(2*seq)+3]) + "/mav0/cam0/data", string(argv[(2*seq)+4]), vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);


    int fps = 20;
    float dT = 1.f/fps;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    // Queue sizes
    const size_t INPUT_QUEUE_SIZE = 20;
    const size_t OUTPUT_QUEUE_SIZE = 50;

    // Process each sequence
    for (seq = 0; seq<num_seq; seq++)
    {
        // Create thread-safe queues for our pipeline
        ThreadSafeQueue<ORB_SLAM3::InputQueueItem> inputQueue(INPUT_QUEUE_SIZE);
        ThreadSafeQueue<ORB_SLAM3::ResultQueueItem> outputQueue(OUTPUT_QUEUE_SIZE);
        
        // Create the feature extractor processor using the factory
        auto featureProcessor = ORB_SLAM3::PipelinedProcessFactory::CreatePipelinedProcess(
            ORB_SLAM3::FeatureExtractorType::ORB, 
            inputQueue, 
            outputQueue
        );
        
        // Start the processor thread
        featureProcessor->StartProcessing();

        // Create a producer thread to read images and fill the input queue
        std::thread producerThread([&vstrImageFilenames, &vTimestampsCam, seq, &nImages, &inputQueue, imageScale, &SLAM]() {
            cv::Mat im;

            // Process all images in the sequence
            for(int ni=0; ni<nImages[seq]; ni++)
            {

                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                // Read image from file
                im = cv::imread(vstrImageFilenames[seq][ni], cv::IMREAD_UNCHANGED);
                double tframe = vTimestampsCam[seq][ni];

                // Check if image is valid
                if(im.empty())
                {
                    cerr << endl << "Failed to load image at: " << vstrImageFilenames[seq][ni] << endl;
                    continue;
                }

                // Resize image if needed
                if(imageScale != 1.f)
                {
                    int width = im.cols * imageScale;
                    int height = im.rows * imageScale;
                    cv::resize(im, im, cv::Size(width, height));
                }else if(SLAM.settings_ && SLAM.settings_->needToResize()){
                    cv::resize(im, im, SLAM.settings_->newImSize());
                }

                // Create input item and enqueue
                ORB_SLAM3::InputQueueItem item;
                item.index = ni;
                item.timestamp = tframe;
                item.filename = vstrImageFilenames[seq][ni];
                item.image = im.clone();
                
                inputQueue.enqueue(item);

                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                double imPush = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

                // Wait to maintain original framerate
                double T = 0;
                if(ni < nImages[seq]-1)
                    T = vTimestampsCam[seq][ni+1] - tframe;
                else if(ni > 0)
                    T = tframe - vTimestampsCam[seq][ni-1];
                
                if(imPush < T)
                    usleep((T-imPush)*1e6);
            }
            
            // Signal that we're done producing items
            inputQueue.shutdown();
        });

        // Consumer (main thread) - process results from the processing thread
        int processedFrames = 0;
        ORB_SLAM3::ResultQueueItem result;
        
        while (outputQueue.dequeue(result)) 
        {
            int ni = result.index;
            double tframe = result.timestamp;
            
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

#ifdef DEBUG_PRINT  
            // Debug visualization of keypoints
            if (std::getenv("DEBUG_KeypointVisualization"))
            {
                // Visualize keypoints on the image
                cv::Mat imWithKeypoints;
                result.image.copyTo(imWithKeypoints);
                
                // Scale down keypoint sizes for visualization
                std::vector<cv::KeyPoint> scaledKeypoints;
                scaledKeypoints.resize(result.keypoints.size());
                int i = 0;
                for (const auto& kp : result.keypoints) {
                    // scaledKeypoints.push_back(kp); // Deep copy each keypoint
                    scaledKeypoints[i] = kp;
                    i++;
                }
                for(auto& kp : scaledKeypoints) {
                    kp.size = 5; // Reduce the size of each keypoint by half
                }
                
                cv::drawKeypoints(result.image, scaledKeypoints, imWithKeypoints, 
                                cv::Scalar(0, 255, 0), 
                                cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); // Use simpler drawing style
                
                // Display the image with keypoints
                cv::imshow("Keypoints", imWithKeypoints);
                cv::waitKey(5); // Wait for 10ms to allow window to update

                std::cout << "[DEBUG] KeypointVisualization  frame=" << ni 
                          << "  keypoints=" << result.keypoints.size() 
                          << std::endl;
                // std::cout << "[DEBUG] KeypointVisualization END" << std::endl;
            }
#endif
            
            // Track using the SLAM system
            SLAM.TrackMonocular(result);
            
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            vTimesTrack[ni] = ttrack;
            
            processedFrames++;
            
            // Print progress
            if (processedFrames % 50 == 0)
            {
                cout << "Processed " << processedFrames << "/" << nImages[seq] << " frames" << endl;
            }
        }

        // Wait for the producer to finish
        producerThread.join();
        
        // Stop the processor
        featureProcessor->Stop();

        // Handle map transitions between sequences
        if(seq < num_seq - 1)
        {
            string kf_file_submap = "./SubMaps/kf_SubMap_" + std::to_string(seq) + ".txt";
            string f_file_submap = "./SubMaps/f_SubMap_" + std::to_string(seq) + ".txt";
            SLAM.SaveTrajectoryEuRoC(f_file_submap);
            SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file_submap);

            cout << "Changing the dataset" << endl;
            SLAM.ChangeDataset();
        }
    }
    
    // Compute timing statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for(size_t ni=0; ni<vTimesTrack.size(); ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    
    cout << endl << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size()/2] * 1000 << " ms" << endl;
    cout << "mean tracking time: " << totaltime/vTimesTrack.size() * 1000 << " ms" << endl;

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file = string(argv[argc-1]) + "KeyFrameTrajectory.txt";
        const string f_file = string(argv[argc-1]) + "CameraTrajectory.txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t*1e-9);
        }
    }
}
