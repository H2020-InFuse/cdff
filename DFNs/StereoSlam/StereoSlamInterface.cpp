/**
 * @addtogroup DFNs
 * @{
 */

#include "StereoSlamInterface.hpp"

namespace CDFF
{
namespace DFN
{

StereoSlamInterface::StereoSlamInterface()
{
    asn1SccFramePair_Initialize(&inImagePair);
    asn1SccTransformWithCovariance_Initialize(&outPose);
}

StereoSlamInterface::~StereoSlamInterface()
{
}

void StereoSlamInterface::framePairInput(const asn1SccFramePair& data)
{
    inImagePair = data;
}

const asn1SccTransformWithCovariance& StereoSlamInterface::poseOutput() const
{
    return outPose;
}

}
}

/** @} */
