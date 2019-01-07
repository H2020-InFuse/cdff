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
    class StereoSlamInterface : public DFNCommonInterface
    {
        public:

            StereoSlamInterface();
            virtual ~StereoSlamInterface();

            /**
             * Send value to input port "imagePair"
             * @param imagePair
             *     This is the current acquisition
             */
            virtual void imagePairInput(const asn1SccFramePair& data);
            /**
             * Query value from output port "Pose"
             * @return Pose
             *     This is the output transform
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
