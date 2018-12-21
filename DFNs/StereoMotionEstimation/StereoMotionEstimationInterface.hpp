/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREOMOTIONESTIMATION_STEREOMOTIONESTIMATIONINTERFACE_HPP
#define STEREOMOTIONESTIMATION_STEREOMOTIONESTIMATIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>
#include <Types/C/TransformWithCovariance.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that computes an estimated motion between two stereo acquisitions
     */
    class StereoMotionEstimationInterface : public DFNCommonInterface
    {
        public:

            StereoMotionEstimationInterface();
            virtual ~StereoMotionEstimationInterface();

            /**
             * Send value to input port "framePair"
             * @param framePair
             *     Degraded/dowscaled input frame pair
             */
            virtual void framePairInput(const asn1SccFramePair& data);
            /**
             * Send value to input port "disparity"
             * @param disparity
             *     Disparity image computed from the input frame pair
             */
            virtual void disparityInput(const asn1SccFrame& data);

            /**
             * Query value from output port "pose"
             * @return pose
             *     Computed pose estimation of the robot
             */
            virtual const asn1SccTransformWithCovariance& poseOutput() const;

        protected:

            asn1SccFramePair inFramePair;
            asn1SccFrame inDisparity;
            asn1SccTransformWithCovariance outPose;
    };
}
}

#endif // STEREOMOTIONESTIMATION_STEREOMOTIONESTIMATIONINTERFACE_HPP

/** @} */
