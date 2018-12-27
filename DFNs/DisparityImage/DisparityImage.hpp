/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYIMAGE_DISPARITYIMAGE_HPP
#define DISPARITYIMAGE_DISPARITYIMAGE_HPP

#include "DisparityImageInterface.hpp"
#include "Helpers/ParametersListHelper.hpp"
#include <Converters/FrameToMatConverter.hpp>

namespace CDFF
{
namespace DFN
{
namespace DisparityImage
{
    /**
     * TODO Class documentation
     */
    class DisparityImage : public DisparityImageInterface
    {
        public:

            DisparityImage();
            virtual ~DisparityImage();

            virtual void configure();
            virtual void process();

            struct DisparityImageParams
            {
                /**
                 * @brief Minimum possible disparity value. Normally, it is zero but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
                 */
                int minDisparity;
                /**
                 * @brief Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation, this parameter must be divisible by 16.
                 */
                int numDisparities;
                /**
                 * @brief Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
                 */
                int blockSize;
                /**
                 * @brief The first parameter controlling the disparity smoothness.
                 */
                int P1;
                /**
                 * @brief The second parameter controlling the disparity smoothness. The larger the values are, the smoother the disparity is. P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels. P2 is the penalty on the disparity change by more than 1 between neighbor pixels. The algorithm requires P2 > P1 . See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown (like 8*number_of_image_channels*SADWindowSize*SADWindowSize and 32*number_of_image_channels*SADWindowSize*SADWindowSize , respectively)
                 */
                int P2;
                /**
                 * @brief Maximum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positive value to disable the check.
                 */
                int disp12MaxDiff;
                /**
                 * @brief Truncation value for the prefiltered image pixels. The algorithm first computes x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval. The result values are passed to the Birchfield-Tomasi pixel cost function.
                 */
                int preFilterCap;
                /**
                 * @brief Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough.
                 */
                int uniquenessRatio;
                /**
                 * @brief Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
                 */
                int speckleWindowSize;
                /**
                 * @brief Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough.
                 */
                int speckleRange;
                /**
                 * @brief Set it to StereoSGBM::MODE_HH to run the full-scale two-pass dynamic programming algorithm. It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and huge for HD-size pictures. By default, it is set to false .
                 */
                int mode;
            };

            Helpers::ParametersListHelper parametersHelper;
            DisparityImageParams parameters;
            static const DisparityImageParams DEFAULT_PARAMETERS;
            Converters::FrameToMatConverter frameToMat;
            void ValidateParameters();
    };
}
}
}

#endif // DISPARITYIMAGE_DISPARITYIMAGE_HPP

/** @} */
