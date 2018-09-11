/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FORCEMESHGENERATOR_FORCEMESHGENERATORINTERFACE_HPP
#define FORCEMESHGENERATOR_FORCEMESHGENERATORINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Frame.h>
#include <PointCloud.hpp>
#include <pcl/io/ply_io.h>

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
            * @param position: end-effector position
            * @param force: end-effector force measurements
            */
            virtual void positionAndForceInput(const asn1SccPosition & position, const asn1SccT_Double & force);

            /**
             * Query value from output port
             * @return point cloud
             */
            virtual const asn1SccPointcloud& pointCloudOutput() const;

        protected:

            asn1SccPose inPose;
            asn1SccPosition inPosition;
            asn1SccT_Double inForce;
            asn1SccPointcloud outPointCloud;

            std::vector<std::pair< pcl::PointXYZ, double > > inPoints;

    };
}
}

#endif // FORCEMESHGENERATOR_FORCEMESHGENERATORINTERFACE_HPP

/** @} */
