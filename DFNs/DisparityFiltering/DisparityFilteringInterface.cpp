/**
 * @addtogroup DFNs
 * @{
 */

#include "DisparityFilteringInterface.hpp"

namespace CDFF
{
namespace DFN
{

DisparityFilteringInterface::DisparityFilteringInterface()
{
    asn1SccFrame_Initialize(&inRawDisparity);
    asn1SccFrame_Initialize(&outFilteredDisparity);
}

DisparityFilteringInterface::~DisparityFilteringInterface()
{
}

void DisparityFilteringInterface::rawDisparityInput(const asn1SccFrame& data)
{
    inRawDisparity = data;
}

const asn1SccFrame& DisparityFilteringInterface::filteredDisparityOutput() const
{
    return outFilteredDisparity;
}

}
}

/** @} */
