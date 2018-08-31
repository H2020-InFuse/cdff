/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef VISUALODOMETRY_LAAS_INTERFACE_HPP
#define VISUALODOMETRY_LAAS_INTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <Frame.h>
#include <TransformWithCovariance.h>

namespace CDFF
{
namespace DFPC
{
    /**
     * Visual odometry which relies on wheel odometry and stereo images developed by LAAS.
     */
    class VisualOdometry_LAASInterface : public DFPCCommonInterface
    {
        public:

            VisualOdometry_LAASInterface();
            virtual ~VisualOdometry_LAASInterface();

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
             * Query value from output port "estimatedMotion"
             * @return estimatedMotion: Pose estimated from the VisualOdometry_LAAS DFPC
             */
            virtual const asn1SccTransformWithCovariance& estimatedMotionOutput() const;


        protected:

            asn1SccFrame inLeftImage;
            asn1SccFrame inRightImage;
            asn1SccTransformWithCovariance outEstimatedMotion;

    };
}
}

#endif //  VISUALODOMETRY_LAAS_INTERFACE_HPP

/** @} */
