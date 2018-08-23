/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef VISUALSLAMSTEREO_INTERFACE_HPP
#define VISUALSLAMSTEREO_INTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <TransformWithCovariance.h>
#include <Frame.h>

namespace CDFF
{
namespace DFPC
{
    /**
     * Simultaneous Localization and Mapping which relies on odometry, stereo images and depths images.
     */
    class VisualSlamStereoInterface : public DFPCCommonInterface
    {
        public:

            VisualSlamStereoInterface();
            virtual ~VisualSlamStereoInterface();

            /**
             * Send value to input port "leftImage"
             * @param leftImage: 
             */
            virtual void leftImageInput(const asn1SccFrame& data);
            /**
             * Send value to input port "rightImage"
             * @param rightImage: 
             */
            virtual void rightImageInput(const asn1SccFrame& data);
            /**
             * Send value to input port "roverPose"
             * @param roverPose: Estimated pose of the rover from Odometry
             */
            virtual void roverPoseInput(const asn1SccTransformWithCovariance& data);

            /**
             * Query value from output port "estimatedPose"
             * @return estimatedPose: Pose estimated from the SLAM DFPC
             */
            virtual const asn1SccTransformWithCovariance& estimatedPoseOutput() const;


        protected:

            asn1SccFrame inLeftImage;
            asn1SccFrame inRightImage;
            asn1SccTransformWithCovariance inRoverPose;
            asn1SccTransformWithCovariance outEstimatedPose;

    };
}

}


#endif //  VISUALSLAMSTEREO_INTERFACE_HPP

/** @} */
