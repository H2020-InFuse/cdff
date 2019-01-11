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
}

StereoSlamInterface::~StereoSlamInterface()
{
}

void StereoSlamInterface::framePairInput(const asn1SccFramePair& data)
{
    inImagePair = data;
}

const asn1SccTransformWithCovariance& StereoSlamInterface::PoseOutput() const
{
    return outPose;
}

}
}

/** @} */
