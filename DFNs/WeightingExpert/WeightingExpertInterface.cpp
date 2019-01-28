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
    asn1SccPosesSequence_Initialize(& inPoses);
    asn1SccPose_Initialize(& outPose);
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
