/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POSEESTIMATOR_POSEESTIMATORINTERFACE_HPP
#define POSEESTIMATOR_POSEESTIMATORINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Sequences.h>
#include <Frame.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that estimates the pose of a robot given the primitives found in the image
     */
    class PoseEstimatorInterface : public DFNCommonInterface
    {
        public:

            PoseEstimatorInterface();
            virtual ~PoseEstimatorInterface();

            /**
             * Send value to input port "image"
             * @param image
             *     2D image captured by a camera
             */
            virtual void imageInput(const asn1SccFrame& data);
            /**
             * Send value to input port "depth"
             * @param depth
             *     2D depth image
             */
            virtual void depthInput(const asn1SccFrame& data);
            /**
             * Send value to input port "primitives"
             * @param primitives
             *     location and necessary information of the found primitives
             */
            virtual void primitivesInput(const asn1SccVectorXdSequence& data);

            /**
             * Query value from output port "poses"
             * @return poses
             *     Pose of the different joints of the robot and its end effector
             */
            virtual const asn1SccPosesSequence& posesOutput() const;

        protected:

            asn1SccFrame inImage;
            asn1SccFrame inDepth;
            asn1SccVectorXdSequence inPrimitives;
            asn1SccPosesSequence outPoses;
    };
}
}

#endif // POSEESTIMATOR_POSEESTIMATORINTERFACE_HPP

/** @} */
