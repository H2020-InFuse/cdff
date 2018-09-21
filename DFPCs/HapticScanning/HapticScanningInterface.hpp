/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef HAPTICSCANNING_INTERFACE_HPP
#define HAPTICSCANNING_INTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <TransformWithCovariance.h>
#include <Frame.h>
#include <PointCloud.hpp>
#include <Sequences.h>

namespace CDFF
{
namespace DFPC
{
    /**
     * Reconstruction of the environment using a force sensor as input
     */
    class HapticScanningInterface : public DFPCCommonInterface
    {
        public:

            HapticScanningInterface();
            virtual ~HapticScanningInterface();

            /**
             * Send value to input port
             * @param pose: estimated rover pose relative to target
             */
            virtual void roverPoseInput(const asn1SccPose& data);

            /**
            * Send value to input port
            * @param positions: end-effector positions
            * @param forces: end-effector force measurements
            */
            virtual void positionAndForceInput(const asn1SccPointsSequence & positions, const asn1SccDoublesSequence & forces);

            /**
             * Query value from output port
             * @return point cloud
             */
            virtual const asn1SccPointcloud& pointCloudOutput() const;

        protected:

            asn1SccPose inRoverPose;
            asn1SccPointsSequence inPositions;
            asn1SccDoublesSequence inForces;
            asn1SccPointcloud outPointCloud;
    };
}

}


#endif //  HAPTICSCANNING_INTERFACE_HPP

/** @} */
