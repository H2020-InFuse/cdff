/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef VISUALODOMETRY_MAG_INTERFACE_HPP
#define VISUALODOMETRY_MAG_INTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <TransformWithCovariance.h>
#include <Frame.h>

namespace dfpc_ci
{
    /**
     * Visual odometry which relies on wheel odometry and stereo images developed by Magellium.
     */
    class VisualOdometry_MAGInterface : public DFPCCommonInterface
    {
        public:

            VisualOdometry_MAGInterface();
            virtual ~VisualOdometry_MAGInterface();

            /**
             * Send value to input port "leftImage"
             * @param leftImage: Left image frame
             */
            virtual void leftImageInput(const asn1SccFrame& data);
            /**
             * Send value to input port "rightImage"
             * @param rightImage: Right image frame
             */
            virtual void rightImageInput(const asn1SccFrame& data);
            /**
             * Send value to input port "odoMotion"
             * @param odoMotion: Estimated pose of the rover from Odometry
             */
            virtual void odoMotionInput(const asn1SccTransformWithCovariance& data);

            /**
             * Query value from output port "estimateMotion"
             * @return estimateMotion: Pose estimated from the VisualOdometry_MAG DFPC
             */
            virtual const asn1SccTransformWithCovariance& estimateMotionOutput() const;


        protected:

            asn1SccFrame inLeftImage;
            asn1SccFrame inRightImage;
            asn1SccTransformWithCovariance inOdoMotion;
            asn1SccTransformWithCovariance outEstimateMotion;

    };
}

#endif //  VISUALODOMETRY_MAG_INTERFACE_HPP

/** @} */
