/**
 * @addtogroup DFNs
 * @{
 */

#include "DepthFilteringInterface.hpp"

namespace CDFF
{
namespace DFN
{

//=====================================================================================================================
DepthFilteringInterface::DepthFilteringInterface() :
inFrame(),
outFrame()
{
}

//=====================================================================================================================
void DepthFilteringInterface::frameInput(const asn1SccFrame& data)
{
    inFrame = data;
}

//=====================================================================================================================
const asn1SccFrame& DepthFilteringInterface::frameOutput() const
{
    return outFrame;
}

}
}

/** @} */
