/**
 * @addtogroup DFNs
 * @{
 */

#ifndef WEIGHTINGEXPERT_WEIGHTINGEXPERTINTERFACE_HPP
#define WEIGHTINGEXPERT_WEIGHTINGEXPERTINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Sequences.h>
#include <Types/C/Pose.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that estimates the pose of a joint given different predictions
     */
    class WeightingExpertInterface : public DFNCommonInterface
    {
        public:

            WeightingExpertInterface();
            virtual ~WeightingExpertInterface();

            /**
             * Send value to input port "poses"
             * @param poses
             *     Poses of a joint of the robot using different methods
             */
            virtual void posesInput(const asn1SccPosesSequence& data);

            /**
             * Query value from output port "pose"
             * @return pose
             *     Estimated pose of the joint
             */
            virtual const asn1SccPose& poseOutput() const;

        protected:

            asn1SccPosesSequence inPoses;
            asn1SccPose outPose;
    };
}
}

#endif // WEIGHTINGEXPERT_WEIGHTINGEXPERTINTERFACE_HPP

/** @} */
