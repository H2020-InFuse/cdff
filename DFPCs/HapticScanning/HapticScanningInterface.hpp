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
             * Send value to input port inputPose
             * @param pose: estimated rover pose relative to target
             */
            virtual void roverPoseInput(const asn1SccPose& pose);

            /**
            * Send value to input port inputPositions
            * @param positions: end-effector positions
            */
            virtual void positionInput(const asn1SccPointSequence & positions);

            /**
            * Send value to input port inputForces
            * @param forces: end-effector force measurements
            */
            virtual void forceInput(const asn1SccDoubleSequence & forces);

            /**
             * Query value from output port outputPointCloud
             * @return point cloud
             */
            virtual const asn1SccPointcloud& pointCloudOutput() const;


        protected:

            asn1SccPose inRoverPose;
            asn1SccPointSequence inPositions;
            asn1SccDoubleSequence inForces;
            asn1SccPointcloud outPointCloud;
    };
}

}


#endif //  HAPTICSCANNING_INTERFACE_HPP

/** @} */
