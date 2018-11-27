/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef HAPTICSCANNING_INTERFACE_HPP
#define HAPTICSCANNING_INTERFACE_HPP

#include "DFPCCommonInterface.hpp"

#include "Types/C/TransformWithCovariance.h"
#include "Types/C/Frame.h"
#include "Types/C/Pointcloud.h"
#include "Types/C/Sequences.h"
#include "Types/C/Wrench.h"

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
            virtual void armBasePoseInput(const asn1SccPose& pose);

            /**
            * Send value to input port inputPositions
            * @param pose: end-effector positions
            */
            virtual void armEndEffectorPoseInput(const asn1SccPose &pose);

            /**
            * Send value to input port inputForces
            * @param wrench: end-effector force measurements
            */
            virtual void armEndEffectorWrenchInput(const asn1SccWrench &wrench);

            /**
             * Query value from output port outputPointCloud
             * @return point cloud
             */
            virtual const asn1SccPointcloud & pointCloudOutput() const;


        protected:

            asn1SccPose inArmBasePose;
            asn1SccPose inArmEndEffectorPose;
            asn1SccWrench inArmEndEffectorWrench;

            asn1SccPointcloud outPointCloud;
    };
}

}


#endif //  HAPTICSCANNING_INTERFACE_HPP

/** @} */
