/**
 * @addtogroup DFPCs
 * @{
 */

#include "VisualSlamStereoInterface.hpp"

namespace CDFF
{
namespace DFPC
{

VisualSlamStereoInterface::VisualSlamStereoInterface() :
inFramePair(),
outEstimatedPose()
{
}

VisualSlamStereoInterface::~VisualSlamStereoInterface()
{
}

void VisualSlamStereoInterface::framePairInput(const asn1SccFramePair& data)
{
    inFramePair = data;
}

const asn1SccTransformWithCovariance& VisualSlamStereoInterface::estimatedPoseOutput() const
{
    return outEstimatedPose;
}

}

}

/** @} */
