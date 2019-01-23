/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYIMAGE_DISPARITYIMAGEINTERFACE_HPP
#define DISPARITYIMAGE_DISPARITYIMAGEINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{
    class DisparityImageInterface : public DFNCommonInterface
    {
        public:

            DisparityImageInterface();
            virtual ~DisparityImageInterface();

            /**
             * Send value to input port "framePair"
             * @param framePair
             *     A stereo pair of images
             */
            virtual void framePairInput(const asn1SccFramePair& data);

            /**
             * Query value from output port "disparity"
             * @return disparity
             *     The corresponding disparity image
             */
            virtual const asn1SccFrame& disparityOutput() const;

        protected:

            asn1SccFramePair inFramePair;
            asn1SccFrame outDisparity;
    };
}
}

#endif // DISPARITYIMAGE_DISPARITYIMAGEINTERFACE_HPP

/** @} */
