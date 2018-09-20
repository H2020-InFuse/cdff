/**
 * @addtogroup DFPCs
 * @{
 */

#include "VisualSlamStereoInterface.hpp"

namespace CDFF
{
namespace DFPC
{

VisualSlamStereoInterface::VisualSlamStereoInterface()
{
}

VisualSlamStereoInterface::~VisualSlamStereoInterface()
{
}

void VisualSlamStereoInterface::leftImageInput(const asn1SccFrame& data)
{
    inLeftImage = data;
}

void VisualSlamStereoInterface::rightImageInput(const asn1SccFrame& data)
{
    inRightImage = data;
}

void VisualSlamStereoInterface::roverPoseInput(const asn1SccTransformWithCovariance& data)
{
    inRoverPose = data;
}

const asn1SccTransformWithCovariance& VisualSlamStereoInterface::estimatedPoseOutput() const
{
    return outEstimatedPose;
}

}

}

/** @} */
