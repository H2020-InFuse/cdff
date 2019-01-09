/**
 * @addtogroup DFNs
 * @{
 */

#ifndef COLORCONVERSION_COLORCONVERSION_HPP
#define COLORCONVERSION_COLORCONVERSION_HPP

#include "ColorConversionInterface.hpp"
#include <opencv2/imgproc.hpp>
#include "Helpers/ParametersListHelper.hpp"

namespace CDFF
{
namespace DFN
{
namespace ColorConversion
{
    /**
     * TODO Class documentation
     */
    class ColorConversion : public ColorConversionInterface
    {
        public:

            ColorConversion();
            virtual ~ColorConversion();

            virtual void configure();
            virtual void process();

            struct ColorConversionParams
            {
                /**
                 * @brief Color mode in which the frame should be converted
                 * 1 = asn1Sccmode_GRAY
                 * 2 = asn1Sccmode_RGB
                 * 3 = asn1Sccmode_RGBA
                 * 4 = asn1Sccmode_BGR
                 * 5 = asn1Sccmode_BGRA
                 * 6 = asn1Sccmode_HSV
                 * 7 = asn1Sccmode_HLS
                 * 8 = asn1Sccmode_YUV
                 * 10 = asn1Sccmode_Lab
                 * 11 = asn1Sccmode_Luv
                 * 12 = asn1Sccmode_XYZ
                 * 13 = asn1Sccmode_YCrCb
                 */
                int targetMode;
            };

            Helpers::ParametersListHelper parametersHelper;
            ColorConversionParams parameters;
            static const ColorConversionParams DEFAULT_PARAMETERS;
            void ValidateParameters();

        private:
            cv::Mat _in;
            cv::Mat _out;

            void convertToGray();
            void convertToRGB();
            void convertToRGBA();
            void convertToBGR();
            void convertToBGRA();
            void convertToHSV();
            void convertToHLS();
            void convertToYUV();
            void convertToLab();
            void convertToLuv();
            void convertToXYZ();
            void convertToYCrCb();
    };
}
}
}

#endif // COLORCONVERSION_COLORCONVERSION_HPP

/** @} */
