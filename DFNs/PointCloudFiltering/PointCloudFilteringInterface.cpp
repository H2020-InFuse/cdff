/**
 * @addtogroup DFNs
 * @{
 */

#include "PointCloudFilteringInterface.hpp"

namespace CDFF
{
namespace DFN
{

PointCloudFilteringInterface::PointCloudFilteringInterface() :
inPointCloud(),
outFilteredPointCloud()
{
}

PointCloudFilteringInterface::~PointCloudFilteringInterface()
{
}

void PointCloudFilteringInterface::pointCloudInput(const asn1SccPointcloud& data)
{
    inPointCloud = data;
}

const asn1SccPointcloud& PointCloudFilteringInterface::filteredPointCloudOutput() const
{
    return outFilteredPointCloud;
}

}
}

/** @} */
