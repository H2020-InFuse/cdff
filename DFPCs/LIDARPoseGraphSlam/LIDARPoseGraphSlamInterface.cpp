/**
 * @addtogroup DFPCs
 * @{
 */

#include "LIDARPoseGraphSlamInterface.hpp"

namespace CDFF
{
namespace DFPC
{

LIDARPoseGraphSlamInterface::LIDARPoseGraphSlamInterface() :
inLidarPointCloud(),
inOdometryPose(),
outPoseEstimate()
{
}

LIDARPoseGraphSlamInterface::~LIDARPoseGraphSlamInterface()
{
}

void LIDARPoseGraphSlamInterface::lidarPointCloudInput(const asn1SccPointcloud& data)
{
    inLidarPointCloud = data;
}

void LIDARPoseGraphSlamInterface::odometryPoseInput(const asn1SccTransformWithCovariance& data)
{
    inOdometryPose = data;
}

const asn1SccTransformWithCovariance& LIDARPoseGraphSlamInterface::poseEstimateOutput() const
{
    return outPoseEstimate;
}

}
}

/** @} */
