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

#ifndef MATCHVISUALIZER_H
#define MATCHVISUALIZER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <string>
#include <vector>
#include <map>

namespace ORB_SLAM3
{

class MatchVisualizer
{
public:
    /**
     * Display matches between two images using OpenCV visualization
     * @param img1 First image
     * @param kp1 Keypoints from first image
     * @param img2 Second image
     * @param kp2 Keypoints from second image
     * @param matches Indices of matching keypoints (kp1[i] matches with kp2[matches[i]])
     * @param windowName Name of the display window
     * @param drawOnly Draw without waiting for keypress (for automated processing)
     * @param spatialSubsample If true, applies spatial subsampling to avoid visual clutter
     * @param gridX Number of grid cells in X direction for spatial subsampling
     * @param gridY Number of grid cells in Y direction for spatial subsampling
     * @param maxPerCell Maximum number of matches to display per grid cell
     */
    static void ShowMatches(
        const cv::Mat& img1, 
        const std::vector<cv::KeyPoint>& kp1, 
        const cv::Mat& img2, 
        const std::vector<cv::KeyPoint>& kp2, 
        const std::vector<int>& matches,
        const std::string& windowName = "Feature Matches",
        bool drawOnly = false,
        bool spatialSubsample = false,
        int gridX = 8,
        int gridY = 6,
        int maxPerCell = 5);

    /**
     * Display matches between two images using OpenCV DMatch format
     * @param img1 First image
     * @param kp1 Keypoints from first image
     * @param img2 Second image
     * @param kp2 Keypoints from second image
     * @param dmatches Vector of DMatch objects
     * @param windowName Name of the display window
     * @param drawOnly Draw without waiting for keypress
     * @param spatialSubsample If true, applies spatial subsampling to avoid visual clutter
     * @param gridX Number of grid cells in X direction for spatial subsampling
     * @param gridY Number of grid cells in Y direction for spatial subsampling
     * @param maxPerCell Maximum number of matches to display per grid cell
     */
    static void ShowMatches(
        const cv::Mat& img1, 
        const std::vector<cv::KeyPoint>& kp1, 
        const cv::Mat& img2, 
        const std::vector<cv::KeyPoint>& kp2, 
        const std::vector<cv::DMatch>& dmatches,
        const std::string& windowName = "Feature Matches",
        bool drawOnly = false,
        bool spatialSubsample = false,
        int gridX = 8,
        int gridY = 6,
        int maxPerCell = 5);

    /**
     * Save match visualization to an image file
     * @param img1 First image
     * @param kp1 Keypoints from first image
     * @param img2 Second image
     * @param kp2 Keypoints from second image
     * @param matches Indices of matching keypoints
     * @param filename Filename to save the image
     * @param spatialSubsample If true, applies spatial subsampling to avoid visual clutter
     * @param gridX Number of grid cells in X direction for spatial subsampling
     * @param gridY Number of grid cells in Y direction for spatial subsampling
     * @param maxPerCell Maximum number of matches to display per grid cell
     */
    static void SaveMatchImage(
        const cv::Mat& img1, 
        const std::vector<cv::KeyPoint>& kp1, 
        const cv::Mat& img2, 
        const std::vector<cv::KeyPoint>& kp2, 
        const std::vector<int>& matches,
        const std::string& filename,
        bool spatialSubsample = true,
        int gridX = 8,
        int gridY = 6,
        int maxPerCell = 5);

private:
    /**
     * Apply spatial subsampling to matches to avoid visual clutter
     * @param kp1 Keypoints from first image
     * @param dmatches Vector of DMatch objects
     * @param imgWidth Width of first image
     * @param imgHeight Height of first image
     * @param gridX Number of grid cells in X direction
     * @param gridY Number of grid cells in Y direction
     * @param maxPerCell Maximum number of matches per grid cell
     * @return Subsampled vector of DMatch objects
     */
    static std::vector<cv::DMatch> SpatialSubsample(
        const std::vector<cv::KeyPoint>& kp1,
        const std::vector<cv::DMatch>& dmatches,
        int imgWidth, int imgHeight,
        int gridX, int gridY, int maxPerCell);
};

} // namespace ORB_SLAM3

#endif // MATCHVISUALIZER_H