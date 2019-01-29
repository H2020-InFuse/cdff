/**
 * @addtogroup DFNs
 * @{
 */

#include "VoxelizationInterface.hpp"

namespace CDFF
{
namespace DFN
{

VoxelizationInterface::VoxelizationInterface() :
inDepth(),
outOctree()
{
    asn1SccFrame_Initialize(& inDepth);
    asn1SccOctree_Initialize(& outOctree);
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
