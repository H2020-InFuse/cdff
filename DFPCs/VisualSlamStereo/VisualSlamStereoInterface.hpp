/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef VISUALSLAMSTEREO_INTERFACE_HPP
#define VISUALSLAMSTEREO_INTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <Types/C/TransformWithCovariance.h>
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFPC
{
    /**
     * Simultaneous Localization and Mapping which relies on stereo images.
     */
    class VisualSlamStereoInterface : public DFPCCommonInterface
    {
        public:

            VisualSlamStereoInterface();
            virtual ~VisualSlamStereoInterface();

            /**
             * Send value to input port "framePair"
             * @param framePair: 
             */
            virtual void framePairInput(const asn1SccFramePair& data);
            /**
             * Query value from output port "estimatedPose"
             * @return estimatedPose: Pose estimated from the SLAM DFPC
             */
            virtual const asn1SccTransformWithCovariance& estimatedPoseOutput() const;


        protected:

            asn1SccFramePair inFramePair;
            asn1SccTransformWithCovariance outEstimatedPose;

    };
}

}


#endif //  VISUALSLAMSTEREO_INTERFACE_HPP

/** @} */
