/**
 * @addtogroup DFNs
 * @{
 */

#include "PoseWeightingInterface.hpp"

namespace CDFF
{
namespace DFN
{

PoseWeightingInterface::PoseWeightingInterface()
{
}

PoseWeightingInterface::~PoseWeightingInterface()
{
}

void PoseWeightingInterface::posesInput(const asn1SccPosesSequence& data)
{
    inPoses = data;
}

const asn1SccPose& PoseWeightingInterface::poseOutput() const
{
    return outPose;
}

}
}

/** @} */
