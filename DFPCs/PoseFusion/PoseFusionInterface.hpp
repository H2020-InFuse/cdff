/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef POSEFUSION_INTERFACE_HPP
#define POSEFUSION_INTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <Types/C/TransformWithCovariance.h>

namespace CDFF
{
namespace DFPC
{
    /**
     * Integrate data corresponding to past poses, and to produce updated past poses after the application of a pose fusion process.
     */
    class PoseFusionInterface : public DFPCCommonInterface
    {
        public:

            PoseFusionInterface();
            virtual ~PoseFusionInterface();

            /**
             * Send value to input port "wheelOdometry"
             * @param wheelOdometry: 
             */
            virtual void wheelOdometryInput(const asn1SccTransformWithCovariance& data);
            /**
             * Send value to input port "visualOdometry"
             * @param visualOdometry: 
             */
            virtual void visualOdometryInput(const asn1SccTransformWithCovariance& data);

            /**
             * Query value from output port "estimatedCurrentPose"
             * @return estimatedCurrentPose: Pose estimated from the SLAM DFPC
             */
            virtual const asn1SccTransformWithCovariance& estimatedCurrentPoseOutput() const;
            /**
             * Query value from output port "estimatedPastPoses"
             * @return estimatedPastPoses: Past pose estimated from the SLAM DFPC
             */
            virtual const asn1SccTransformWithCovariance& estimatedPastPosesOutput() const;


        protected:

            asn1SccTransformWithCovariance inWheelOdometry;
            asn1SccTransformWithCovariance inVisualOdometry;
            asn1SccTransformWithCovariance outEstimatedCurrentPose;
            asn1SccTransformWithCovariance outEstimatedPastPoses;

    };
}
}
#endif //  POSEFUSION_INTERFACE_HPP

/** @} */
