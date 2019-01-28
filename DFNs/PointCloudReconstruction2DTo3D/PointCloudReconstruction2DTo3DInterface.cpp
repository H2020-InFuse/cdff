/**
 * @addtogroup DFNs
 * @{
 */

#include "PointCloudReconstruction2DTo3DInterface.hpp"

namespace CDFF
{
namespace DFN
{

PointCloudReconstruction2DTo3DInterface::PointCloudReconstruction2DTo3DInterface()
{
    asn1SccCorrespondenceMap2D_Initialize(& inMatches);
    asn1SccPose_Initialize(& inPose);
    asn1SccPointcloud_Initialize(& outPointcloud);
}

PointCloudReconstruction2DTo3DInterface::~PointCloudReconstruction2DTo3DInterface()
{
}

void PointCloudReconstruction2DTo3DInterface::matchesInput(const asn1SccCorrespondenceMap2D& data)
{
    inMatches = data;
}

void PointCloudReconstruction2DTo3DInterface::poseInput(const asn1SccPose& data)
{
    inPose = data;
}

const asn1SccPointcloud& PointCloudReconstruction2DTo3DInterface::pointcloudOutput() const
{
    return outPointcloud;
}

}
}

/** @} */
