/**
 * @addtogroup DFPCs
 * @{
 */

#include "PoseFusionInterface.hpp"

namespace CDFF
{
namespace DFPC
{

PoseFusionInterface::PoseFusionInterface()
{
    asn1SccTransformWithCovariance_Initialize(& inWheelOdometry);
    asn1SccTransformWithCovariance_Initialize(& inVisualOdometry);
    asn1SccTransformWithCovariance_Initialize(& outEstimatedCurrentPose);
    asn1SccTransformWithCovariance_Initialize(& outEstimatedPastPoses);
}

PoseFusionInterface::~PoseFusionInterface()
{
}

void PoseFusionInterface::wheelOdometryInput(const asn1SccTransformWithCovariance& data)
{
    inWheelOdometry = data;
}

void PoseFusionInterface::visualOdometryInput(const asn1SccTransformWithCovariance& data)
{
    inVisualOdometry = data;
}

const asn1SccTransformWithCovariance& PoseFusionInterface::estimatedCurrentPoseOutput() const
{
    return outEstimatedCurrentPose;
}

const asn1SccTransformWithCovariance& PoseFusionInterface::estimatedPastPosesOutput() const
{
    return outEstimatedPastPoses;
}

}
}

/** @} */
