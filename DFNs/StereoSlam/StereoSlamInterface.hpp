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
     * If tracking is successful, the pose output is relative to the pose
     * of the first image passed to the tracker.
     */
    class StereoSlamInterface : public DFNCommonInterface
    {
        public:

            StereoSlamInterface();
            virtual ~StereoSlamInterface();

            /**
             * Send value to input port "imagePair"
             * @param imagePair The current image on which to perform tracking.
             */
            virtual void imagePairInput(const asn1SccFramePair& data);
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
