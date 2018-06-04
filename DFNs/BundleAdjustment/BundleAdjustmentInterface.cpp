/**
 * @addtogroup DFNs
 * @{
 */

#include "BundleAdjustmentInterface.hpp"

namespace dfn_ci
{

BundleAdjustmentInterface::BundleAdjustmentInterface()
{
}

BundleAdjustmentInterface::~BundleAdjustmentInterface()
{
}

void BundleAdjustmentInterface::framesSequenceInput(const asn1SccFramesSequence& data)
{
    inFramesSequence = data;
}

const asn1SccPosesSequence& BundleAdjustmentInterface::posesSequenceOutput() const
{
    return outPosesSequence;
}

bool BundleAdjustmentInterface::successOutput() const
{
    return outSuccess;
}

}

/** @} */
