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
: inRawDisparity(), outFilteredDisparity()
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
