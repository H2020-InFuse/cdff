/**
 * @addtogroup DFPCs
 * @{
 */

#include "VisualSlamInterface.hpp"

namespace dfpc_ci
{

VisualSlamInterface::VisualSlamInterface()
{
}

VisualSlamInterface::~VisualSlamInterface()
{
}

void VisualSlamInterface::depthImageInput(const asn1SccFrame& data)
{
    inDepthImage = data;
}

void VisualSlamInterface::rgbImageInput(const asn1SccFrame& data)
{
    inRgbImage = data;
}

void VisualSlamInterface::leftImageInput(const asn1SccFrame& data)
{
    inLeftImage = data;
}

void VisualSlamInterface::rightImageInput(const asn1SccFrame& data)
{
    inRightImage = data;
}

void VisualSlamInterface::roverPoseInput(const asn1SccTransformWithCovariance& data)
{
    inRoverPose = data;
}

const asn1SccTransformWithCovariance& VisualSlamInterface::estimatedPoseOutput() const
{
    return outEstimatedPose;
}

}

/** @} */
