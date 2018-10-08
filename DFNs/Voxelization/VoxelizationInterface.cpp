/**
 * @addtogroup DFNs
 * @{
 */

#include "VoxelizationInterface.hpp"

namespace CDFF
{
namespace DFN
{

VoxelizationInterface::VoxelizationInterface()
{
}

VoxelizationInterface::~VoxelizationInterface()
{
}

void VoxelizationInterface::depthInput(const asn1SccFrame& data)
{
    inDepth = data;
}

const asn1SccOctree& VoxelizationInterface::octreeOutput() const
{
    return outOctree;
}

}
}

/** @} */
