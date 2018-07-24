/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef VISUALSLAM_INTERFACE_HPP
#define VISUALSLAM_INTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <Frame.h>
#include <TransformWithCovariance.h>

namespace dfpc_ci
{
    /**
     * Simultaneous Localization and Mapping which relies on odometry, stereo images and depths images.
     */
    class VisualSlamInterface : public DFPCCommonInterface
    {
        public:

            VisualSlamInterface();
            virtual ~VisualSlamInterface();

            /**
             * Send value to input port "depthImage"
             * @param depthImage: 
             */
            virtual void depthImageInput(const asn1SccFrame& data);
            /**
             * Send value to input port "rgbImage"
             * @param rgbImage: 
             */
            virtual void rgbImageInput(const asn1SccFrame& data);
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

            asn1SccFrame inDepthImage;
            asn1SccFrame inRgbImage;
            asn1SccFrame inLeftImage;
            asn1SccFrame inRightImage;
            asn1SccTransformWithCovariance inRoverPose;
            asn1SccTransformWithCovariance outEstimatedPose;

    };
}

#endif //  VISUALSLAM_INTERFACE_HPP

/** @} */
