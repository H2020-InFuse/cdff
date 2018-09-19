/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FORCEMESHGENERATOR_FORCEMESHGENERATORINTERFACE_HPP
#define FORCEMESHGENERATOR_FORCEMESHGENERATORINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <PointCloud.hpp>
#include <Sequences.h>

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

#endif // FORCEMESHGENERATOR_FORCEMESHGENERATORINTERFACE_HPP

/** @} */
