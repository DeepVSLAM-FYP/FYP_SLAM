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

#include "MatchVisualizer.h"
#include "Frame.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <algorithm>

namespace ORB_SLAM3
{

std::vector<cv::DMatch> MatchVisualizer::SpatialSubsample(
    const std::vector<cv::KeyPoint>& kp1,
    const std::vector<cv::DMatch>& dmatches,
    int imgWidth, int imgHeight,
    int gridX, int gridY, int maxPerCell)
{
    // Create a map to store matches for each grid cell
    std::map<std::pair<int,int>, std::vector<cv::DMatch>> cellMap;

    // Assign each match to a grid cell based on the query keypoint location
    for (const auto& m : dmatches) {
        const auto& pt = kp1[m.queryIdx].pt;
        int cx = static_cast<int>(pt.x * gridX / imgWidth);
        int cy = static_cast<int>(pt.y * gridY / imgHeight);
        
        // Ensure within grid bounds
        cx = std::min(cx, gridX - 1);
        cy = std::min(cy, gridY - 1);
        
        cellMap[{cx,cy}].push_back(m);
    }

    // Sample matches from each cell, prioritizing those with better scores (lower distance)
    std::vector<cv::DMatch> sampledMatches;
    sampledMatches.reserve(cellMap.size() * maxPerCell); // Reserve approximate size

    for (auto& kv : cellMap) {
        auto& matches = kv.second;
        
        // Sort matches by distance (lower is better)
        std::sort(matches.begin(), matches.end(),
                 [](const cv::DMatch& a, const cv::DMatch& b) {
                     return a.distance < b.distance;
                 });
        
        // Take up to maxPerCell best matches from this cell
        const int count = std::min(static_cast<int>(matches.size()), maxPerCell);
        for (int i = 0; i < count; ++i) {
            sampledMatches.push_back(matches[i]);
        }
    }

    return sampledMatches;
}

void MatchVisualizer::ShowMatches(
    const cv::Mat& img1, 
    const std::vector<cv::KeyPoint>& kp1, 
    const cv::Mat& img2, 
    const std::vector<cv::KeyPoint>& kp2, 
    const std::vector<int>& matches,
    const std::string& windowName,
    bool drawOnly,
    bool spatialSubsample,
    int gridX,
    int gridY,
    int maxPerCell)
{
    // Convert matches to DMatch format
    std::vector<cv::DMatch> dmatches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i] >= 0) {
            cv::DMatch m;
            m.queryIdx = i;
            m.trainIdx = matches[i];
            m.distance = 1.0; // Just a placeholder value
            dmatches.push_back(m);
        }
    }

    // Call the overloaded version that uses DMatch
    ShowMatches(img1, kp1, img2, kp2, dmatches, windowName, drawOnly, 
                spatialSubsample, gridX, gridY, maxPerCell);
}

void MatchVisualizer::ShowMatches(
    const cv::Mat& img1, 
    const std::vector<cv::KeyPoint>& kp1, 
    const cv::Mat& img2, 
    const std::vector<cv::KeyPoint>& kp2, 
    const std::vector<cv::DMatch>& dmatches,
    const std::string& windowName,
    bool drawOnly,
    bool spatialSubsample,
    int gridX,
    int gridY,
    int maxPerCell)
{
    // Create a new output image
    cv::Mat img_matches;
    
    // Ensure images are in color for drawing colored matches
    cv::Mat img1_color, img2_color;
    if (img1.channels() == 1) {
        cv::cvtColor(img1, img1_color, cv::COLOR_GRAY2BGR);
    } else {
        img1_color = img1.clone();
    }
    
    if (img2.channels() == 1) {
        cv::cvtColor(img2, img2_color, cv::COLOR_GRAY2BGR);
    } else {
        img2_color = img2.clone();
    }
    
    // Spatial subsampling if requested
    std::vector<cv::DMatch> matchesToDraw = dmatches;
    if (spatialSubsample && !dmatches.empty()) {
        matchesToDraw = SpatialSubsample(kp1, dmatches, img1.cols, img1.rows, 
                                         gridX, gridY, maxPerCell);
    }
    
    // Draw matches
    cv::drawMatches(
        img1_color, kp1,
        img2_color, kp2,
        matchesToDraw,
        img_matches,
        cv::Scalar_<int>::all(-1) ,   // Match color (random)
        cv::Scalar(0, 0, 255),   // Single point color (red) for keypoints without match
        std::vector<char>(),     // Mask (empty for all matches)
        cv::DrawMatchesFlags::DEFAULT
    );
    
    // Display match statistics
    std::string stats = "Total Matches: " + std::to_string(dmatches.size());
    if (spatialSubsample) {
        stats += " | Displayed: " + std::to_string(matchesToDraw.size());
    }
    
    cv::putText(img_matches, stats, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
    
    // Display the image
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, img_matches);
    
    // Wait for keyboard input if not in "draw only" mode
    if (!drawOnly) {
        cv::waitKey(0);
    }
}

void MatchVisualizer::SaveMatchImage(
    const cv::Mat& img1, 
    const std::vector<cv::KeyPoint>& kp1, 
    const cv::Mat& img2, 
    const std::vector<cv::KeyPoint>& kp2, 
    const std::vector<int>& matches,
    const std::string& filename,
    bool spatialSubsample,
    int gridX,
    int gridY,
    int maxPerCell)
{
    // Convert matches to DMatch format
    std::vector<cv::DMatch> dmatches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i] >= 0) {
            cv::DMatch m;
            m.queryIdx = i;
            m.trainIdx = matches[i];
            m.distance = 1.0; // Just a placeholder
            dmatches.push_back(m);
        }
    }
    
    // Create output image
    cv::Mat img_matches;
    
    // Ensure images are in color
    cv::Mat img1_color, img2_color;
    if (img1.channels() == 1) {
        cv::cvtColor(img1, img1_color, cv::COLOR_GRAY2BGR);
    } else {
        img1_color = img1.clone();
    }
    
    if (img2.channels() == 1) {
        cv::cvtColor(img2, img2_color, cv::COLOR_GRAY2BGR);
    } else {
        img2_color = img2.clone();
    }
    
    // Apply spatial subsampling if requested
    std::vector<cv::DMatch> matchesToDraw = dmatches;
    if (spatialSubsample && !dmatches.empty()) {
        matchesToDraw = SpatialSubsample(kp1, dmatches, img1.cols, img1.rows, 
                                         gridX, gridY, maxPerCell);
    }
    
    // Draw matches
    cv::drawMatches(
        img1_color, kp1,
        img2_color, kp2,
        matchesToDraw,
        img_matches,
        cv::Scalar_<int>::all(-1) ,   // Match color (random)
        cv::Scalar(0, 0, 255),   // Single point color (red)
        std::vector<char>(),     // Mask (empty)
        cv::DrawMatchesFlags::DEFAULT
    );
    
    // Add match statistics
    std::string stats = "Total Matches: " + std::to_string(dmatches.size());
    if (spatialSubsample) {
        stats += " | Displayed: " + std::to_string(matchesToDraw.size());
    }
    
    cv::putText(img_matches, stats, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
    
    // Save the image
    try {
        cv::imwrite(filename, img_matches);
        std::cout << "Match visualization saved to " << filename << std::endl;
    } catch (const cv::Exception& e) {
        std::cerr << "Error saving match visualization: " << e.what() << std::endl;
    }
}



void MatchVisualizer::ShowMatchedKeypoints(
    const cv::Mat& img,
    const std::vector<cv::KeyPoint>& keypoints,
    const std::vector<int>& matchedIndices,
    const std::string& windowName,
    bool drawOnly,
    const cv::Scalar& matchColor,
    const cv::Scalar& unmatchedColor)
{
    // Create a working copy of the input image
    cv::Mat imgDisplay;
    
    // Ensure the image is in color for drawing colored keypoints
    if (img.channels() == 1) {
        cv::cvtColor(img, imgDisplay, cv::COLOR_GRAY2BGR);
    } else {
        imgDisplay = img.clone();
    }
    
    // Create a mask for marking matched keypoints
    std::vector<bool> isMatched(keypoints.size(), false);
    for (int idx : matchedIndices) {
        if (idx >= 0 && idx < static_cast<int>(keypoints.size())) {
            isMatched[idx] = true;
        }
    }
    
    // Draw all keypoints
    for (size_t i = 0; i < keypoints.size(); i++) {
        // Choose color based on whether the keypoint is matched
        const cv::Scalar& color = isMatched[i] ? matchColor : unmatchedColor;
        
        // Draw the keypoint
        cv::circle(imgDisplay, 
                  keypoints[i].pt, 
                  4, // have a constant radius
                  color, 
                  2); // thickness
        
        // Draw orientation line if available
        // if (keypoints[i].angle >= 0) {
        //     float angleRad = keypoints[i].angle * (M_PI / 180.0f);
        //     cv::Point2f direction(cos(angleRad), sin(angleRad));
        //     cv::Point2f end = keypoints[i].pt + (direction * keypoints[i].size);
        //     cv::line(imgDisplay, keypoints[i].pt, end, color, 2);
        // }
    }
    
    // Add statistics
    int matchCount = matchedIndices.size();
    int totalKeypoints = keypoints.size();
    std::string stats = "Keypoints: " + std::to_string(totalKeypoints) + 
                        " | Matched: " + std::to_string(matchCount);
    
    cv::putText(imgDisplay, stats, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
    
    // Display the image
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, imgDisplay);
    
    // Wait for keyboard input if not in "draw only" mode
    if (!drawOnly) {
        cv::waitKey(0);
    }
}

void MatchVisualizer::ShowFrameMatches(
    const Frame& lastFrame, 
    const Frame& currentFrame,
    const std::string& windowName,
    int waitTime)
{
    std::vector<cv::DMatch> matches;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    
    // Convert MapPoints to keypoints and create matches
    for(size_t i = 0; i < currentFrame.mvpMapPoints.size(); i++)
    {
        if(currentFrame.mvpMapPoints[i])
        {
            // Use the actual index in the keypoints arrays
            cv::DMatch match;
            match.queryIdx = keypoints1.size(); // Current index in keypoints1
            match.trainIdx = keypoints2.size(); // Current index in keypoints2
            matches.push_back(match);
            
            // Check if the index is within bounds of the keypoint arrays
            if(i < lastFrame.mvKeysUn.size() && i < currentFrame.mvKeysUn.size()) {
                keypoints1.push_back(lastFrame.mvKeysUn[i]);
                keypoints2.push_back(currentFrame.mvKeysUn[i]);
            }
            else {
                // Remove the match we just added since we can't add the keypoints
                matches.pop_back();
            }
        }
    }
    
    // If no matches were found, display a message and return
    if(matches.empty()) {
        std::cout << "No matches to display between frames" << std::endl;
        return;
    }
    
    // Draw the matches
    cv::Mat outImg;
    cv::drawMatches(lastFrame.image, keypoints1, currentFrame.image, keypoints2, matches, outImg);
    
    // Add statistics
    std::string stats = "Matched MapPoints: " + std::to_string(matches.size());
    cv::putText(outImg, stats, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
    
    // Display the image
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, outImg);
    cv::waitKey(waitTime);
}

} // namespace ORB_SLAM3