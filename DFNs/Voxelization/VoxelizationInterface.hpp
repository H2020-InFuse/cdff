/**
 * @addtogroup DFNs
 * @{
 */

#ifndef VOXELIZATION_VOXELIZATIONINTERFACE_HPP
#define VOXELIZATION_VOXELIZATIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Frame.h>
#include <Octree.h>


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
            ~VoxelizationInterface() = default;

            /**
             * Send value to input port inputFrame
             * @param frame: 2D depth image
             */
            virtual void frameInput(const asn1SccFrame& data);

            /**
             * Query value from output port ouputOctree
             * @return octree
             */
            virtual const asn1SccOctree& octreeOutput() const;

        protected:

            asn1SccFrame inFrame;
            asn1SccOctree outOctree;
    };
}
}

#endif // VOXELIZATION_VOXELIZATIONINTERFACE_HPP

/** @} */
