/**
 * @addtogroup DFPCs
 * @{
 */

#include "VisualSlamDepthInterface.hpp"

namespace CDFF
{
namespace DFPC
{

VisualSlamDepthInterface::VisualSlamDepthInterface()
{
    asn1SccFrame_Initialize(& inDepthImage);
    asn1SccFrame_Initialize(& inRgbImage);
    asn1SccTransformWithCovariance_Initialize(& inRoverPose);
    asn1SccTransformWithCovariance_Initialize(& outEstimatedPose)
}

VisualSlamDepthInterface::~VisualSlamDepthInterface()
{
}

void VisualSlamDepthInterface::depthImageInput(const asn1SccFrame& data)
{
    inDepthImage = data;
}

void VisualSlamDepthInterface::rgbImageInput(const asn1SccFrame& data)
{
    inRgbImage = data;
}

void VisualSlamDepthInterface::roverPoseInput(const asn1SccTransformWithCovariance& data)
{
    inRoverPose = data;
}

const asn1SccTransformWithCovariance& VisualSlamDepthInterface::estimatedPoseOutput() const
{
    return outEstimatedPose;
}

}
}

/** @} */
