/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREODEGRADATION_STEREODEGRADATIONINTERFACE_HPP
#define STEREODEGRADATION_STEREODEGRADATIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that degrade the resolution of an image
     */
    class StereoDegradationInterface : public DFNCommonInterface
    {
        public:

            StereoDegradationInterface();
            virtual ~StereoDegradationInterface();

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

#endif // STEREODEGRADATION_STEREODEGRADATIONINTERFACE_HPP

/** @} */
