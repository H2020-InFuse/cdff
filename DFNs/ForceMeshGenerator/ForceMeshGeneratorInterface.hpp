/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FORCEMESHGENERATOR_FORCEMESHGENERATORINTERFACE_HPP
#define FORCEMESHGENERATOR_FORCEMESHGENERATORINTERFACE_HPP

#include "DFNCommonInterface.hpp"

#include <memory>

#include "Types/C/Pointcloud.h"
#include "Types/C/Sequences.h"
#include "Types/C/Wrench.h"

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that creates a point cloud using the feedback from a force sensor
     */
    class ForceMeshGeneratorInterface : public DFNCommonInterface
    {
        public:

            ForceMeshGeneratorInterface();
            ~ForceMeshGeneratorInterface() override = default;

            /**
             * Send value to input port armBasePose
             * @param pose: estimated rover pose relative to target
             */
            virtual void armBasePoseInput(const asn1SccPose &pose);

            /**
            * Send value to input port armEndEffectorWrench
            * @param pose: end-effector positions
            */
            virtual void armEndEffectorPoseInput(const asn1SccPose &pose);

            /**
            * Send value to input port armEndEffectorWrench
            * @param wrench: end-effector force measurements
            */
            virtual void armEndEffectorWrenchInput(const asn1SccWrench wrench);

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

#endif // FORCEMESHGENERATOR_FORCEMESHGENERATORINTERFACE_HPP

/** @} */
