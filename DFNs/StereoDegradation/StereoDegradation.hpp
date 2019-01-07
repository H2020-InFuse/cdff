/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEPAIRDEGRADATION_IMAGEPAIRDEGRADATION_HPP
#define IMAGEPAIRDEGRADATION_IMAGEPAIRDEGRADATION_HPP

#include "ImagePairDegradationInterface.hpp"
#include "Helpers/ParametersListHelper.hpp"

namespace CDFF
{
namespace DFN
{
namespace ImagePairDegradation
{
    /**
     * @brief Implementation of the resolution degradation algorithms provided by OpenCV
     * See OpenCV's documentation on cv::resize() for more information
     * https://docs.opencv.org/3.4.0/da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d
     */
    class ImagePairDegradation : public ImagePairDegradationInterface
    {
        public:

            ImagePairDegradation();
            virtual ~ImagePairDegradation();

            virtual void configure();
            virtual void process();

            struct ImagePairDegradationParams
            {
                /**
                 * @brief Degradation ratio to be applied over the x-axis
                 */
                int xratio;

                /**
                 * @brief Degradation ratio to be applied over the y-axis
                 */
                int yratio;

                /**
                 * @brief Interpolation method
                 * See OpenCV's documentation on enum cv::InterpolationFlags for more information
                 * https://docs.opencv.org/3.4.0/da/d54/group__imgproc__transform.html#ga5bb5a1fea74ea38e1a5445ca803ff121
                 */
                int method;
            };

            Helpers::ParametersListHelper parametersHelper;
            ImagePairDegradationParams parameters;
            static const ImagePairDegradationParams DEFAULT_PARAMETERS;
            void ValidateParameters();
    };
}
}
}

#endif // IMAGEPAIRDEGRADATION_IMAGEPAIRDEGRADATION_HPP

/** @} */
