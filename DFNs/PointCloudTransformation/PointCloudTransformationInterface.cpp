/**
 * @addtogroup DFNs
 * @{
 */

#include "PointCloudTransformationInterface.hpp"

namespace CDFF
{
namespace DFN
{

PointCloudTransformationInterface::PointCloudTransformationInterface() :
inPointCloud(),
inPose(),
outTransformedPointCloud()
{
}

PointCloudTransformationInterface::~PointCloudTransformationInterface()
{
}

void PointCloudTransformationInterface::pointCloudInput(const asn1SccPointcloud& data)
{
    inPointCloud = data;
}

void PointCloudTransformationInterface::poseInput(const asn1SccPose& data)
{
    inPose = data;
}

const asn1SccPointcloud& PointCloudTransformationInterface::transformedPointCloudOutput() const
{
    return outTransformedPointCloud;
}

}
}

/** @} */
