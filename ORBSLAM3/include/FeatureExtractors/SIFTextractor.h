#pragma once
#include "FeatureExtractorBase.h"
#include <opencv2/features2d.hpp>

namespace ORB_SLAM3 {

class SIFTextractor : public FeatureExtractor {
public:
    /// We ignore the 'scaleFactor' and 'nlevels' parameters: this extractor
    //This is only done for compatibility with the final intent of the code -- implementing DL based feature extractors
    /// If you want to use features at multiple scales, modify the implementation
    /// always reports exactly 1 level (level 0).
    explicit SIFTextractor(int nfeatures);

    /// keypoint detection + descriptor computation
    int operator()(const cv::Mat& image, cv::InputArray mask,
                   std::vector<cv::KeyPoint>& kps,
                   cv::Mat& desc,
                   std::vector<int>& vLap) override;

    // ————————————————————————————————————————————————————————————
    // Fake a one‐level pyramid everywhere:
    int  GetLevels()                     const override { return 1;            }
    float GetScaleFactor()               const override { return 1.0f;         }
    const std::vector<float>& GetScaleFactors()       const override { return sf_;    }
    const std::vector<float>& GetInverseScaleFactors()const override { return isf_;   }
    const std::vector<float>& GetScaleSigmaSquares()  const override { return ss2_;   }
    const std::vector<float>& 
          GetInverseScaleSigmaSquares() const override { return iss2_;  }
    const std::vector<cv::Mat>& GetImagePyramid()     const override { return pyr_;  }

    // descriptor traits
    int  descriptorType()   const override { return CV_32F;         }
    int  descriptorLength() const override { return 128;            }
    NormType norm()         const override { return NormType::L2;   }

private:
    cv::Ptr<cv::SIFT> sift_;

    // all of these vectors will be length=1
    std::vector<float> sf_{1.0f};
    std::vector<float> isf_{1.0f};
    std::vector<float> ss2_{1.0f};
    std::vector<float> iss2_{1.0f};

    // single‐element pyramid
    mutable std::vector<cv::Mat> pyr_{};
};

}  // namespace ORB_SLAM3
