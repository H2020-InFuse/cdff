/**
 * @addtogroup DFNs
 * @{
 */

#ifndef VOXELIZATION_VOXELIZATIONINTERFACE_HPP
#define VOXELIZATION_VOXELIZATIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Frame.h>
#include <PointCloud.hpp>


typedef struct{
    double resolution;
    PointCloudWrapper::PointCloudConstPtr point_cloud;
} asn1SccT_Octree;


namespace CDFF
{
namespace DFN
{
    /**
     * DFN that voxelizes a 2D depth map
     */
    class VoxelizationInterface : public DFNCommonInterface
    {
        public:

            VoxelizationInterface();

            /**
             * Send value to input port
             * @param frame: 2D depth image
             */
            virtual void frameInput(const asn1SccFrame& data);

            /**
             * Query value from output port
             * @return octree
             */
            virtual const asn1SccT_Octree& octreeOutput() const;

        protected:

            asn1SccFrame inFrame;
            asn1SccT_Octree outOctree;
    };
}
}

#endif // VOXELIZATION_VOXELIZATIONINTERFACE_HPP

/** @} */
