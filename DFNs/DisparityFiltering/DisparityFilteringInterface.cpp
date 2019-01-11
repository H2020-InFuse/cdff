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
