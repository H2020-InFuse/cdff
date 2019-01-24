/**
 * @addtogroup DFNs
 * @{
 */

#include "PointCloudTransformInterface.hpp"

namespace CDFF
{
namespace DFN
{

PointCloudTransformInterface::PointCloudTransformInterface()
{
    asn1SccPointcloud_Initialize(& inPointCloud);
    asn1SccPose_Initialize(& inPose);
    asn1SccPointcloud_Initialize(& outTransformedPointCloud);
}

PointCloudTransformInterface::~PointCloudTransformInterface()
{
}

void PointCloudTransformInterface::pointCloudInput(const asn1SccPointcloud& data)
{
    inPointCloud = data;
}

void PointCloudTransformInterface::poseInput(const asn1SccPose& data)
{
    inPose = data;
}

const asn1SccPointcloud& PointCloudTransformInterface::transformedPointCloudOutput() const
{
    return outTransformedPointCloud;
}

}
}

/** @} */
