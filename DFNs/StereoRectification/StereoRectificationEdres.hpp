/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREORECTIFICATION_STEREORECTIFICATIONEDRES_HPP
#define STEREORECTIFICATION_STEREORECTIFICATIONEDRES_HPP

#include "StereoRectificationInterface.hpp"
#include "edres-wrapper/EdresRectification.h"
#include "Helpers/ParametersListHelper.hpp"

namespace CDFF
{
namespace DFN
{
namespace StereoRectification
{
    /**
     * @brief Implementation of the rectification process provided by EDRES library
     */
    class StereoRectificationEdres : public StereoRectificationInterface
    {
        public:

            StereoRectificationEdres();
            virtual ~StereoRectificationEdres();

            virtual void configure();
            virtual void process();

            struct StereoRectificationEdresParams
            {
                /**
                 * @brief Path to the correction map files, without filename extension
                 * There should be two files named "sensor_name.dx" & "sensor_name.dy"
                 * If these files are located in the directory "path/to/files",
                 * this parameter should be set to "path/to/files/sensor_name"
                 */
                std::string mapFileLeft;

                /**
                 * @brief Path to the correction map files, without filename extension
                 * There should be two files named "sensor_name.dx" & "sensor_name.dy"
                 * If these files are located in the directory "path/to/files",
                 * this parameter should be set to "path/to/files/sensor_name"
                 */
                std::string mapFileRight;

                /**
                 * @brief Degradation ratio to be applied over the x-axis
                 */
                int xratio;

                /**
                 * @brief Degradation ratio to be applied over the y-axis
                 */
                int yratio;

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
            StereoRectificationEdresParams parameters;
            static const StereoRectificationEdresParams DEFAULT_PARAMETERS;
            void ValidateParameters();

        private:
            Edres::StereoRectification *_rectification;
            std::string _mapFileLeft;
            std::string _mapFileRight;
            bool _initialized;

    };
}
}
}

#endif // STEREORECTIFICATION_STEREORECTIFICATIONEDRES_HPP

/** @} */
