/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEDEGRADATION_IMAGEDEGRADATIONINTERFACE_HPP
#define IMAGEDEGRADATION_IMAGEDEGRADATIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Frame.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that degrade the resolution of an image
     */
    class ImageDegradationInterface : public DFNCommonInterface
    {
        public:

            ImageDegradationInterface();
            virtual ~ImageDegradationInterface();

            /**
             * Send value to input port "originalImage"
             * @param originalImage
             *     Full size image
             */
            virtual void originalImageInput(const asn1SccFrame& data);

            /**
             * Query value from output port "degradedImage"
             * @return degradedImage
             *     Degraded/dowscaled image
             */
            virtual const asn1SccFrame& degradedImageOutput() const;

        protected:

            asn1SccFrame inOriginalImage;
            asn1SccFrame outDegradedImage;
    };
}
}

#endif // IMAGEDEGRADATION_IMAGEDEGRADATIONINTERFACE_HPP

/** @} */
