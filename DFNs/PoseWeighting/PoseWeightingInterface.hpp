/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POSEWEIGHTING_POSEWEIGHTINGINTERFACE_HPP
#define POSEWEIGHTING_POSEWEIGHTINGINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Sequences.h>
#include <Types/C/Pose.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that estimates the pose of an element given different predictions. This DFN contains a Kalman Filter and hence it considers the current inputs to the DFN as well as the previous ones.
     */
    class PoseWeightingInterface : public DFNCommonInterface
    {
        public:

            PoseWeightingInterface();
            virtual ~PoseWeightingInterface();

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

#endif // POSEWEIGHTING_POSEWEIGHTINGINTERFACE_HPP

/** @} */
