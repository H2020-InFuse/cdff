/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEPAIRDEGRADATION_IMAGEPAIRDEGRADATIONINTERFACE_HPP
#define IMAGEPAIRDEGRADATION_IMAGEPAIRDEGRADATIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that degrade the resolution of an image
     */
    class ImagePairDegradationInterface : public DFNCommonInterface
    {
        public:

            ImagePairDegradationInterface();
            virtual ~ImagePairDegradationInterface();

            /**
             * Send value to input port "originalImagePair"
             * @param originalImagePair
             *     Full size image pair
             */
            virtual void originalImagePairInput(const asn1SccFramePair& data);

            /**
             * Query value from output port "degradedImagePair"
             * @return degradedImagePair
             *     Degraded/dowscaled image pair
             */
            virtual const asn1SccFramePair& degradedImagePairOutput() const;

        protected:

            asn1SccFramePair inOriginalImagePair;
            asn1SccFramePair outDegradedImagePair;
    };
}
}

#endif // IMAGEPAIRDEGRADATION_IMAGEPAIRDEGRADATIONINTERFACE_HPP

/** @} */
