/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYIMAGE_DISPARITYIMAGE_HPP
#define DISPARITYIMAGE_DISPARITYIMAGE_HPP

#include "DisparityImageInterface.hpp"
#include "Helpers/ParametersListHelper.hpp"

#include <opencv2/calib3d.hpp>

#if WITH_XIMGPROC
#include <opencv2/ximgproc.hpp>
#endif

namespace CDFF
{
namespace DFN
{
namespace DisparityImage
{
    /**
     * @brief Implementation of the stereo correspondance algorithms provided by OpenCV
     * See OpenCV's documentation on cv::StereoMatcher, cv::StereoBM and cv::StereoSGBM for more information
     * https://docs.opencv.org/3.4.0/d2/d6e/classcv_1_1StereoMatcher.html
     * https://docs.opencv.org/3.4.0/d9/dba/classcv_1_1StereoBM.html
     * https://docs.opencv.org/3.4.0/d2/d85/classcv_1_1StereoSGBM.html
     *
     * If OpenCV contrib module ximgproc is installed, this DFN also implement a disparity filtering algorithm
     * See OpenCV's documentation on cv::ximgproc::DisparityWLSFilter
     * https://docs.opencv.org/3.4.0/d9/d51/classcv_1_1ximgproc_1_1DisparityWLSFilter.html
     */
    class DisparityImage : public DisparityImageInterface
    {
        public:

            DisparityImage();
            virtual ~DisparityImage();

            virtual void configure();
            virtual void process();

            struct stereoBMParams
            {
                /**
                 * @brief Pre-Filter Type
                 * 0 = PREFILTER_NORMALIZED_RESPONSE
                 * 1 = PREFILTER_XSOBEL
                 */
                int preFilterType;

                /**
                 * @brief Size of the Pre-Filter
                 * It must be an odd number >= 5 & <= 255
                 */
                int preFilterSize;

                /**
                 * @brief Texture Threshold
                 */
                int textureThreshold;
            };

            struct stereoSGBMParams
            {
                /**
                 * @brief The first parameter controlling the disparity smoothness.
                 */
                int P1;

                /**
                 * @brief The second parameter controlling the disparity smoothness.
                 * The larger the values are, the smoother the disparity is.
                 * P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels.
                 * P2 is the penalty on the disparity change by more than 1 between neighbor pixels.
                 * The algorithm requires P2 > P1 .
                 * See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown
                 * (like 8*number_of_image_channels*blockSize*blockSize
                 * and 32*number_of_image_channels*blockSize*blockSize , respectively)
                 */
                int P2;

                /**
                 * @brief Algorithm mode.
                 * 0 = MODE_SGBM
                 * 1 = MODE_HH
                 * 2 = MODE_SGBM_3WAY
                 * 3 = MODE_HH4
                 * Set it to StereoSGBM::MODE_HH to run the full-scale two-pass dynamic programming algorithm.
                 * It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and huge for HD-size pictures.
                 */
                int mode;
            };

            struct stereoMatcherParams
            {
                /**
                 * @brief Algorithm to be used
                 * 0 = Block Matching (BM)
                 * 1 = Semi Global Block Matching (SGBM)
                 */
                int algorithm;

                /**
                 * @brief Minimum possible disparity value.
                 * Normally, it is zero but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
                 */
                int minDisparity;

                /**
                 * @brief Maximum disparity minus minimum disparity.
                 * The value is always greater than zero.
                 * In the current implementation, this parameter must be divisible by 16.
                 */
                int numDisparities;

                /**
                 * @brief Matched block size. It must be an odd number >=1 .
                 * Normally, it should be somewhere in the 3..11 range.
                 */
                int blockSize;

                /**
                 * @brief Maximum size of smooth disparity regions to consider their noise speckles and invalidate.
                 * Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
                 */
                int speckleWindowSize;

                /**
                 * @brief Maximum disparity variation within each connected component.
                 * If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16.
                 * Normally, 1 or 2 is good enough.
                 */
                int speckleRange;

                /**
                 * @brief Maximum allowed difference (in integer pixel units) in the left-right disparity check.
                 * Set it to a non-positive value to disable the check.
                 */
                int disp12MaxDiff;

                /**
                 * @brief Truncation value for the prefiltered image pixels.
                 * The algorithm first computes x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval.
                 * The result values are passed to the Birchfield-Tomasi pixel cost function.
                 */
                int preFilterCap;

                /**
                 * @brief Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct.
                 * Normally, a value within the 5-15 range is good enough.
                 */
                int uniquenessRatio;

                stereoBMParams bmParams;
                stereoSGBMParams sgbmParams;
            };

            struct filterParams
            {
                /**
                 * @brief Set to true to filter the disparity map
                 */
                bool useFilter;

                /**
                 * @brief Filtering with confidence requires two disparity maps (for the left and right views) and is approximately two times slower.
                 * However, quality is typically significantly better.
                 */
                bool useConfidence;

                /** @brief DepthDiscontinuityRadius is a parameter used in confidence computation.
                 * It defines the size of low-confidence regions around depth discontinuities.
                 */
                int depthDiscontinuityRadius;

                /** @brief Lambda is a parameter defining the amount of regularization during filtering.
                 * Larger values force filtered disparity map edges to adhere more to source image edges.
                 * Typical value is 8000.
                 */
                double lambda;

                /** @brief LRCthresh is a threshold of disparity difference used in left-right-consistency check during
                 * confidence map computation. The default value of 24 (1.5 pixels) is virtually always good enough.
                 */
                int lrcThresh;

                /** @brief SigmaColor is a parameter defining how sensitive the filtering process is to source image edges.
                 * Large values can lead to disparity leakage through low-contrast edges.
                 * Small values can make the filter too sensitive to noise and textures in the source image.
                 * Typical values range from 0.8 to 2.0.
                 */
                double sigmaColor;
            };

            struct DisparityImageParams
            {
                stereoMatcherParams stereoMatcher;
#if WITH_XIMGPROC
                filterParams filter;
#endif
            };

            Helpers::ParametersListHelper parametersHelper;
            DisparityImageParams parameters;
            static const DisparityImageParams DEFAULT_PARAMETERS;
            void ValidateParameters();

        private:
            cv::Ptr<cv::StereoBM> _bm;
            cv::Ptr<cv::StereoSGBM> _sgbm;
#if WITH_XIMGPROC
            int _algorithm;
            bool _useConfidence;
            cv::Ptr<cv::ximgproc::DisparityWLSFilter> _filter;
            cv::Ptr<cv::StereoMatcher> _rightMatcher;
#endif
    };
}
}
}

#endif // DISPARITYIMAGE_DISPARITYIMAGE_HPP

/** @} */
