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
        : inDispImage(), outPointCloud()
{
    asn1SccFramePair_Initialize(&inDispImage);
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
