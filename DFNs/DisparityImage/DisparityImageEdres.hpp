/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYIMAGE_DISPARITYIMAGEEDRES_HPP
#define DISPARITYIMAGE_DISPARITYIMAGEEDRES_HPP

#include "DisparityImageInterface.hpp"
#include "Helpers/ParametersListHelper.hpp"

namespace CDFF
{
namespace DFN
{
namespace DisparityImage
{
    /**
     * @brief Implementation of the disparity algorithms provided by EDRES library
     */
    class DisparityImageEdres : public DisparityImageInterface
    {
        public:

            DisparityImageEdres();
            virtual ~DisparityImageEdres();

            virtual void configure();
            virtual void process();

            struct DisparityParams{
                /**
                 * @brief Points minimum distance (disparity of closer points will return errValue)
                 */
                double minDistance;
                /**
                 * @brief Points maximum distance (disparity of farther points will return errValue)
                 */
                double maxDistance;
                /**
                 * @brief Stereo-correlation method to be used
                 * 0 = LUMIN:    Correlation of levels of Grey, @warning Implies Disparities on PixelDepth = FLOAT
                 * 1 = GRADLINE: Correlation of gradients, line algorithm
                 * 2 = GRADBAND: Correlation of gradients, band algorithm
                 * 3 = FAST:     Fast stereo algorithm, @note no band algorithm available in this case
                 * 4 = AW:       Adaptive weight approach @todo Investigate this method
                 */
                int method;
                /**
                 * @brief Type of gradient to be used
                 */
                int grad;
                /**
                 * @brief Pixel depth of the gradient image to be used
                 */
                int gradType;
                /**
                 * @brief Pixel depth of the disparity image output
                 */
                int dispType;
            };

            struct FilterParams{
                /**
                 * @brief Set to true to filter the disparity image
                 */
                bool filter;

                /**
                 * @brief Trimming width in pixels
                 */
                int   trimWidth;

                /**
                 * @brief Connexity threshold
                 */
                float connexityThresh;

                /**
                 * @brief Minimum surface of areas to be kept
                 */
                int   surfMin;

                /**
                 * @brief Maximum surface of areas to be kept
                 */
                int   surfMax;
            };

            struct DisparityImageEdresParams
            {
                DisparityParams disparityParams;
                FilterParams filterParams;

            };

            Helpers::ParametersListHelper parametersHelper;
            DisparityImageEdresParams parameters;
            static const DisparityImageEdresParams DEFAULT_PARAMETERS;
            void ValidateParameters();
    };
}
}
}

#endif // DISPARITYIMAGE_DISPARITYIMAGEEDRES_HPP

/** @} */
