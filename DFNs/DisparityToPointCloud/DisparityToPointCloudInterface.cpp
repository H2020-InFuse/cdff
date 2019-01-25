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
    asn1SccFrame_Initialize(&inDispImage);
    asn1SccPointcloud_Initialize(&outPointCloud);
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
