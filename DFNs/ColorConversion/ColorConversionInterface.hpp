/**
 * @addtogroup DFNs
 * @{
 */

#ifndef COLORCONVERSION_COLORCONVERSIONINTERFACE_HPP
#define COLORCONVERSION_COLORCONVERSIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that convert an image from a colorspace to another (i.e. RGB to Grayscale or HSV to BGR)
     */
    class ColorConversionInterface : public DFNCommonInterface
    {
        public:

            ColorConversionInterface();
            virtual ~ColorConversionInterface();

            /**
             * Send value to input port "originalImage"
             * @param originalImage
             *     Original image
             */
            virtual void originalImageInput(const asn1SccFrame& data);

            /**
             * Query value from output port "convertedImage"
             * @return convertedImage
             *     Converted image
             */
            virtual const asn1SccFrame& convertedImageOutput() const;

        protected:

            asn1SccFrame inOriginalImage;
            asn1SccFrame outConvertedImage;
    };
}
}

#endif // COLORCONVERSION_COLORCONVERSIONINTERFACE_HPP

/** @} */
