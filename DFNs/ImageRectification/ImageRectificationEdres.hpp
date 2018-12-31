/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGERECTIFICATION_IMAGERECTIFICATIONEDRES_HPP
#define IMAGERECTIFICATION_IMAGERECTIFICATIONEDRES_HPP

#include "ImageRectificationInterface.hpp"
#include "edres-wrapper/EdresRectification.h"
#include "Helpers/ParametersListHelper.hpp"

namespace CDFF
{
namespace DFN
{
namespace ImageRectification
{
    /**
     * @brief Implementation of the rectification process provided by EDRES library
     */
    class ImageRectificationEdres : public ImageRectificationInterface
    {
        public:

            ImageRectificationEdres();
            virtual ~ImageRectificationEdres();

            virtual void configure();
            virtual void process();

            struct ImageRectificationEdresParams
            {
                /**
                 * @brief Path to the correction map files, without filename extension
                 * There should be two files named "sensor_name.dx" & "sensor_name.dy"
                 * If these files are located in the directory "path/to/files",
                 * this parameter should be set to "path/to/files/sensor_name"
                 */
                std::string mapFile;

                /**
                 * @brief Degradation ratio to be applied over the x-axis
                 */
                int xratio;

                /**
                 * @brief Degradation ratio to be applied over the y-axis
                 */
                int yratio;

                /**
                 * @brief Shift to apply to the output image along the x-axis (in pixels)
                 */
                int xshift;

                /**
                 * @brief Shift to apply to the output image along the y-axis (in pixels)

                 */
                int yshift;

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
            ImageRectificationEdresParams parameters;
            static const ImageRectificationEdresParams DEFAULT_PARAMETERS;
            void ValidateParameters();

        private:
            Edres::Correction _correction;
            std::string _mapFile;
    };
}
}
}

#endif // IMAGERECTIFICATION_IMAGERECTIFICATIONEDRES_HPP

/** @} */
