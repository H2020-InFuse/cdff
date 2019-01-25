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
DepthFilteringInterface::DepthFilteringInterface()
{
    asn1SccFrame_Initialize(&inFrame);
    asn1SccFrame_Initialize(&outFrame);
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
