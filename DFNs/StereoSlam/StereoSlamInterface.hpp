/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREOSLAM_STEREOSLAMINTERFACE_HPP
#define STEREOSLAM_STEREOSLAMINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/TransformWithCovariance.h>
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{
/**
     * @brief DFN that performs visual SLAM on a stereo image pair input.
     * As this is a relative localisation technique, if tracking is successful,
     * the pose output is expressed in the reference frame of the first image passed to the tracker.
     */
    class StereoSlamInterface : public DFNCommonInterface
    {
        public:

            StereoSlamInterface();
            virtual ~StereoSlamInterface();

            /**
             * Send value to input port "framePair"
             * @param data The current rectifed image pair on which to perform tracking.
             */
            virtual void framePairInput(const asn1SccFramePair& data);
            /**
             * Query value from output port "Pose"
             * @return Pose The latest pose estimated by the SLAM system.
             * It is expressed relative to the pose of the first image.
             */
            virtual const asn1SccTransformWithCovariance& PoseOutput() const;

        protected:
            asn1SccFramePair inImagePair;
            asn1SccTransformWithCovariance outPose;
    };
}
}

#endif // STEREOSLAM_STEREOSLAMINTERFACE_HPP

/** @} */
