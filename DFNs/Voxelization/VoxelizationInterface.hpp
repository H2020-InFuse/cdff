/**
 * @addtogroup DFNs
 * @{
 */

#ifndef VOXELIZATION_VOXELIZATIONINTERFACE_HPP
#define VOXELIZATION_VOXELIZATIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Octree.h>
#include <Frame.h>

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
            virtual ~VoxelizationInterface();

            /**
             * Send value to input port "depth"
             * @param depth
             *     depth map
             */
            virtual void depthInput(const asn1SccFrame& data);

            /**
             * Query value from output port "octree"
             * @return octree
             *     octree created from the input depth map
             */
            virtual const asn1SccOctree& octreeOutput() const;

        protected:

            asn1SccFrame inDepth;
            asn1SccOctree outOctree;
    };
}
}

#endif // VOXELIZATION_VOXELIZATIONINTERFACE_HPP

/** @} */
