/**
 * @addtogroup DFNs
 * @{
 */

#include "WeightingExpertInterface.hpp"

namespace CDFF
{
namespace DFN
{

WeightingExpertInterface::WeightingExpertInterface()
{
}

WeightingExpertInterface::~WeightingExpertInterface()
{
}

void WeightingExpertInterface::posesInput(const asn1SccPosesSequence& data)
{
    inPoses = data;
}

const asn1SccPose& WeightingExpertInterface::poseOutput() const
{
    return outPose;
}

}
}

/** @} */
