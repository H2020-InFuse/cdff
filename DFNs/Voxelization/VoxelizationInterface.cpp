/**
 * @addtogroup DFNs
 * @{
 */

#include "VoxelizationInterface.hpp"

namespace CDFF
{
namespace DFN
{

//=====================================================================================================================
VoxelizationInterface::VoxelizationInterface()
{
}

//=====================================================================================================================
void VoxelizationInterface::frameInput(const asn1SccFrame& data)
{
    inFrame = data;
}

//=====================================================================================================================
const asn1SccOctree& VoxelizationInterface::octreeOutput() const
{
    return outOctree;
}

}
}

/** @} */
