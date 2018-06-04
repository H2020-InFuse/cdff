/**
 * @addtogroup DFNs
 * @{
 */

#ifndef BUNDLEADJUSTMENT_INTERFACE_HPP
#define BUNDLEADJUSTMENT_INTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <FramesSequence.hpp>
#include <PosesSequence.hpp>

namespace dfn_ci
{
    /**
     * DFN that determines the poses of a camera from the images taken at those poses.
     */
    class BundleAdjustmentInterface : public DFNCommonInterface
    {
        public:

            BundleAdjustmentInterface();
            virtual ~BundleAdjustmentInterface();

            /**
             * Send value to input port "framesSequence"
             * @param framesSequence: sequence of 2D images captured by the same camera
             */
            virtual void framesSequenceInput(const asn1SccFramesSequence& data);

            /**
             * Query value from output port "posesSequence"
             * @return posesSequence: poses of the camera in the coordinate system relative
             * to the pose of the camera in the first picture.
             */
            virtual const asn1SccPosesSequence& posesSequenceOutput() const;

            /**
             * Query value from output port "success"
             * @return success: whether the estimation of the poses was successful. 
             * If this variable is false the poses sequence is meaningless.
             */
            virtual bool successOutput() const;

        protected:

            asn1SccFramesSequence inFramesSequence;
            asn1SccPosesSequence outPosesSequence;
            bool outSuccess;
    };
}

#endif // BUNDLEADJUSTMENT_INTERFACE_HPP

/** @} */
