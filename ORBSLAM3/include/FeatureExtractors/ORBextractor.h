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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>
#include "FeatureExtractorBase.h"


namespace ORB_SLAM3
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor : public FeatureExtractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    int operator()(const cv::Mat& image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::Mat& descriptors, std::vector<int>& vLappingArea) override;
    inline int GetLevels() const override {return nlevels;}
    inline float GetScaleFactor() const override {return scaleFactor;}
    inline const std::vector<float>& GetScaleFactors() const override {return mvScaleFactor;}
    inline const std::vector<float>& GetInverseScaleFactors() const override {return mvInvScaleFactor;}
    inline const std::vector<float>& GetScaleSigmaSquares() const override {return mvLevelSigma2;}
    inline const std::vector<float>& GetInverseScaleSigmaSquares() const override {return mvInvLevelSigma2;}
    inline const std::vector<cv::Mat>& GetImagePyramid() const override {return mvImagePyramid;}

    int descriptorType() const override { return CV_8U; }
    int descriptorLength() const override { return 32; } // ORB uses 32 bytes
    NormType norm() const override { return NormType::HAMMING; }

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;

    int nfeatures;
    // given by the user typically 1.2
    double scaleFactor;

    // number of levels in the scale pyramid typically 8
    int nlevels;
    int iniThFAST;
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    // Scale factors for the different levels, mvScaleFactor[level_i] = scaleFactor^i
    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    

    //mvLevelSigma2[level_i] = mvScaleFactor[level_i]^2
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    // Image pyramid moved to private use GetImagePyramid() to access it
    std::vector<cv::Mat> mvImagePyramid;
};

} //namespace ORB_SLAM

#endif

