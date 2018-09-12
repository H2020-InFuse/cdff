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
