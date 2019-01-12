/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREORECTIFICATION_STEREORECTIFICATIONINTERFACE_HPP
#define STEREORECTIFICATION_STEREORECTIFICATIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that rectifies an stereo pair
     */
    class StereoRectificationInterface : public DFNCommonInterface
    {
        public:

            StereoRectificationInterface();
            virtual ~StereoRectificationInterface();

            /**
             * Send value to input port "originalStereoPair"
             * @param originalStereoPair
             *     Original distorted stereo pair
             */
            virtual void originalStereoPairInput(const asn1SccFramePair& data);

            /**
             * Query value from output port "rectifiedStereoPair"
             * @return rectifiedStereoPair
             *     Undistorted/rectified stereo pair
             */
            virtual const asn1SccFramePair& rectifiedStereoPairOutput() const;

        protected:

            asn1SccFramePair inOriginalStereoPair;
            asn1SccFramePair outRectifiedStereoPair;
    };
}
}

#endif // STEREORECTIFICATION_STEREORECTIFICATIONINTERFACE_HPP

/** @} */
