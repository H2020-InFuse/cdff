/**
 * @addtogroup DFNs
 * @{
 */

#include "DisparityToPointCloudInterface.hpp"

namespace CDFF
{
namespace DFN
{

DisparityToPointCloudInterface::DisparityToPointCloudInterface()
{
}

DisparityToPointCloudInterface::~DisparityToPointCloudInterface()
{
}

void DisparityToPointCloudInterface::dispImageInput(const asn1SccFrame& data)
{
    inDispImage = data;
}

const asn1SccPointcloud& DisparityToPointCloudInterface::pointCloudOutput() const
{
    return outPointCloud;
}

}
}

/** @} */
