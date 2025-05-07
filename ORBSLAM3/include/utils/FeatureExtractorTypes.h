//utils/FeatureExtractorTypes.h
#pragma once
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


namespace ORB_SLAM3 {

    //Struct to hold the input queue item to the  feature extractor pipeline
    struct InputQueueItem {
        InputQueueItem() : index(0), timestamp(0.0) {}
        InputQueueItem(
            size_t idx, const double &ts, 
            const std::string &imgName, const cv::Mat &img): index(idx), timestamp(ts), name(imgName), image(img) {}
        size_t index;
        const double &timestamp;
        std::string name;
        cv::Mat image;
    };


    struct ResultQueueItem {

        ResultQueueItem() : index(0), timestamp(0.0) {}
        ResultQueueItem(
            size_t idx, const double &ts, 
            const std::string &imgName, const cv::Mat &img): index(idx), timestamp(ts), name(imgName), image(img) {}

        size_t index;
        const double &timestamp;
        std::string name;
        cv::Mat image;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
    };













}