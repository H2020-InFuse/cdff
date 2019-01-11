/**
 * @addtogroup DFNs
 * @{
 */

#include "DisparityToPointCloudWithIntensityInterface.hpp"

namespace CDFF
{
namespace DFN
{

DisparityToPointCloudWithIntensityInterface::DisparityToPointCloudWithIntensityInterface()
{
}

DisparityToPointCloudWithIntensityInterface::~DisparityToPointCloudWithIntensityInterface()
{
}

void DisparityToPointCloudWithIntensityInterface::dispImageInput(const asn1SccFrame& data)
{
    inDispImage = data;
}

void DisparityToPointCloudWithIntensityInterface::intensityImageInput(const asn1SccFrame& data)
{
    inIntensityImage = data;
}

const asn1SccPointcloud& DisparityToPointCloudWithIntensityInterface::pointCloudOutput() const
{
    return outPointCloud;
}

}
}

/** @} */
