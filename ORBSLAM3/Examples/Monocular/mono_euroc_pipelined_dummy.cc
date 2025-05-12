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
#include <filesystem>

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
    if (argc < 6)
    {
        cerr << endl
            << "Usage: ./mono_euroc_pipelined_dummy path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 path_to_feature_directory (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (output_trajectory_file_dir)" << endl;
        return 1;
    }

    string featureDir = string(argv[5]);
    cout << "Using features from: " << featureDir << endl;

    const int num_seq = (argc - 4) / 2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName = (((argc - 4) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc - 1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector<vector<string>> vstrImageFilenames;
    vector<vector<double>> vTimestampsCam;
    vector<int> nImages;

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq < num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
        LoadImages(string(argv[(2 * seq) + 3]) + "/mav0/cam0/data", string(argv[(2 * seq) + 4]), vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl
        << "-------" << endl;
    cout.precision(17);

    int fps = 20;
    float dT = 1.f / fps;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    // Queue sizes
    const size_t INPUT_QUEUE_SIZE = 20;
    const size_t OUTPUT_QUEUE_SIZE = 50;

    // Process each sequence
    for (seq = 0; seq < num_seq; seq++)
    {
        // Create thread-safe queues for our pipeline
        ThreadSafeQueue<ORB_SLAM3::InputQueueItem> inputQueue(INPUT_QUEUE_SIZE);
        ThreadSafeQueue<ORB_SLAM3::ResultQueueItem> outputQueue(OUTPUT_QUEUE_SIZE);

        // Create the dummy feature extractor processor using the factory
        auto featureProcessor = ORB_SLAM3::PipelinedProcessFactory::CreatePipelinedProcess(
            ORB_SLAM3::FeatureExtractorType::SUPERPOINT,
            inputQueue,
            outputQueue,
            featureDir);

        // Start the processor thread
        featureProcessor->StartProcessing();

        // Create a producer thread to read images and fill the input queue
        std::thread producerThread([&vstrImageFilenames, &vTimestampsCam, seq, &nImages, &inputQueue, imageScale, &SLAM]()
                                {
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
            inputQueue.shutdown(); });

        // Consumer (main thread) - process results from the processing thread
        int processedFrames = 0;
        ORB_SLAM3::ResultQueueItem result;

        while (outputQueue.dequeue(result))
        {
            int ni = result.index;
            double tframe = result.timestamp;

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            // resize keypoints if needed
            if (SLAM.settings_ && SLAM.settings_->needToResize())
            {
                cv::Size newSize = SLAM.settings_->newImSize();
                cv::Size originalSize = SLAM.settings_->originalImSize();
                // Extract points from keypoints
                std::vector<cv::Point2f> points;
                cv::KeyPoint::convert(result.keypoints, points);

                // Pre-calculate scaling factors to avoid redundant calculations
                float scaleX = (float)newSize.width / originalSize.width;
                float scaleY = (float)newSize.height / originalSize.height;

                // Create new keypoints with scaled positions
                std::vector<cv::KeyPoint> newKeypoints;
                for (size_t i = 0; i < result.keypoints.size(); i++)
                {
                    cv::KeyPoint kp = result.keypoints[i];
                    float scaledX = points[i].x * scaleX;
                    float scaledY = points[i].y * scaleY;
                    kp.pt = cv::Point2f(scaledX, scaledY);
                    newKeypoints.push_back(kp);
                }
                result.keypoints = newKeypoints;
            }

            // Filter out keypoints that are outside the image boundaries
            const int imgWidth = result.image.cols;
            const int imgHeight = result.image.rows;
            
            std::vector<cv::KeyPoint> validKeypoints;
            cv::Mat validDescriptors;
            
            int removedCount = 0;
            if (!result.keypoints.empty() && !result.descriptors.empty()) {
                validDescriptors.create(0, result.descriptors.cols, result.descriptors.type());
                
                for (size_t i = 0; i < result.keypoints.size(); i++) {
                    const auto& kp = result.keypoints[i];
                    
                    // Check if keypoint is within image boundaries
                    if (kp.pt.x >= 0 && kp.pt.x < imgWidth && 
                        kp.pt.y >= 0 && kp.pt.y < imgHeight) {
                        validKeypoints.push_back(kp);
                        validDescriptors.push_back(result.descriptors.row(i));
                    } else {
                        removedCount++;
                    }
                }
                
                // Update result with filtered keypoints and descriptors
                result.keypoints = validKeypoints;
                result.descriptors = validDescriptors;
                
                if (removedCount > 0 && std::getenv("DEBUG_FEAT") != nullptr) {
                    std::cout << "[DEBUG_FEAT] Removed " << removedCount 
                            << " keypoints outside image boundaries after resizing. Remaining: " 
                            << validKeypoints.size() << std::endl;
                }
            }

            // Debug visualization of keypoints
            if (std::getenv("DEBUG_KeypointVisualization"))
            {
                // Visualize keypoints on the image
                cv::Mat imWithKeypoints;
                result.image.copyTo(imWithKeypoints);
                
                // Scale down keypoint sizes for visualization
                std::vector<cv::KeyPoint> scaledKeypoints;
                scaledKeypoints.reserve(result.keypoints.size());
                
                // Check if keypoints are within image dimensions
                const int imgWidth = result.image.cols;
                const int imgHeight = result.image.rows;
                int i = 0;
                for (const auto& kp : result.keypoints) {
                    scaledKeypoints.push_back(kp);
                    // Check if keypoint is within image boundaries
                    if (!(kp.pt.x >= 0 && kp.pt.x < imgWidth && 
                        kp.pt.y >= 0 && kp.pt.y < imgHeight)) {
                        std::cout << "frame=" << ni << "  keypoint idx= " << i
                                << "  kp.pt.x= " << kp.pt.x << "  kp.pt.y= " << kp.pt.y 
                                << "  imgWidth= " << imgWidth << "  imgHeight= " << imgHeight << std::endl;
                    }
                    i++;
                }
                
                for(auto& kp : scaledKeypoints) {
                    kp.size = 5; // Set fixed size for all keypoints
                }
                
                cv::drawKeypoints(result.image, scaledKeypoints, imWithKeypoints, 
                                cv::Scalar(0, 255, 0), 
                                cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                
                // Display the image with keypoints
                cv::imshow("Keypoints", imWithKeypoints);
                cv::waitKey(5); // Wait for 10ms to allow window to update

                // std::cout << "[DEBUG] KeypointVisualization  frame=" << ni 
                //         << "  keypoints=" << result.keypoints.size() 
                //         << std::endl;
                // std::cout << "[DEBUG] KeypointVisualization END" << std::endl;
            } 

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
        if (seq < num_seq - 1)
        {
            // Create submaps directory if it doesn't exist
            std::filesystem::path submapsDir("./SubMaps");
            if (!std::filesystem::exists(submapsDir))
            {
                std::filesystem::create_directory(submapsDir);
                std::cout << "Created directory: " << submapsDir << std::endl;
            }

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
    for (size_t ni = 0; ni < vTimesTrack.size(); ni++)
    {
        totaltime += vTimesTrack[ni];
    }

    cout << endl
        << "-------" << endl
        << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size() / 2] * 1000 << " ms" << endl;
    cout << "mean tracking time: " << totaltime / vTimesTrack.size() * 1000 << " ms" << endl;

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file = string(argv[argc - 1]) + "KeyFrameTrajectory.txt";
        const string f_file = string(argv[argc - 1]) + "CameraTrajectory.txt";
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
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t * 1e-9);
        }
    }
}