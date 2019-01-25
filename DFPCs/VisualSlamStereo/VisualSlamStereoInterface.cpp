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
    asn1SccFrame_Initialize(& inLeftImage);
    asn1SccFrame_Initialize(& inRightImage);
    asn1SccTransformWithCovariance_Initialize(& inRoverPose);
    asn1SccTransformWithCovariance_Initialize(& outEstimatedPose);
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
