/**
 * @addtogroup DFNs
 * @{
 */

#include "StereoMotionEstimationInterface.hpp"

namespace CDFF
{
namespace DFN
{

StereoMotionEstimationInterface::StereoMotionEstimationInterface()
{
}

StereoMotionEstimationInterface::~StereoMotionEstimationInterface()
{
}

void StereoMotionEstimationInterface::framePairInput(const asn1SccFramePair& data)
{
    inFramePair = data;
}

void StereoMotionEstimationInterface::disparityInput(const asn1SccFrame& data)
{
    inDisparity = data;
}

const asn1SccTransformWithCovariance& StereoMotionEstimationInterface::poseOutput() const
{
    return outPose;
}

}
}

/** @} */
