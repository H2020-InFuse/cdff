/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEDEGRADATION_IMAGEDEGRADATIONEDRES_HPP
#define IMAGEDEGRADATION_IMAGEDEGRADATIONEDRES_HPP

#include "ImageDegradationInterface.hpp"
#include "Helpers/ParametersListHelper.hpp"

namespace CDFF
{
namespace DFN
{
namespace ImageDegradation
{
    /**
     * @brief Implementation of the resolution degradation algorithms provided by EDRES library
     */
    class ImageDegradationEdres : public ImageDegradationInterface
    {
        public:

            ImageDegradationEdres();
            virtual ~ImageDegradationEdres();

            virtual void configure();
            virtual void process();

            struct ImageDegradationEdresParams
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
                 * @brief Degradation method to be used
                 * 0 = MEAN: Resolution degradation by using the mean value of the pixels
                 * 1 = BINNING: Resolution degradation by software binning (sum of the value of the pixels).
                 */
                int method;

                /**
                 * @brief Pixel depth of the asn1SccFrame output
                 * 5 = BYTE: One byte per pixel, Char alignement, 256 levels (8U)
                 * 6 = INT16: One 16 bits signed integer per pixel (16S)
                 * 7 = INT32: One 32 bits signed integer per pixel (32S)
                 * 8 = FLOAT: One float per pixel (32F)
                 */
                int outType;
            };

            Helpers::ParametersListHelper parametersHelper;
            ImageDegradationEdresParams parameters;
            static const ImageDegradationEdresParams DEFAULT_PARAMETERS;
            void ValidateParameters();
    };
}
}
}

#endif // IMAGEDEGRADATION_IMAGEDEGRADATIONEDRES_HPP

/** @} */
