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


typedef struct {
    int nCount;
    asn1SccPoint arr[100000];
} asn1SccPointArray;

typedef struct {
    int nCount;
    asn1SccT_Double arr[100000];
} asn1SccDoubleArray;


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
            virtual void positionAndForceInput(const asn1SccPointArray & positions, const asn1SccDoubleArray & forces);

            /**
             * Query value from output port
             * @return point cloud
             */
            virtual const asn1SccPointcloud& pointCloudOutput() const;

        protected:

            asn1SccPose inRoverPose;
            asn1SccPointArray inPositions;
            asn1SccDoubleArray inForces;
            asn1SccPointcloud outPointCloud;
    };
}
}

#endif // FORCEMESHGENERATOR_FORCEMESHGENERATORINTERFACE_HPP

/** @} */
