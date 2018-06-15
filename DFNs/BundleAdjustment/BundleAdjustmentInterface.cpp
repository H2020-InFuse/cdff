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

void BundleAdjustmentInterface::correspondenceMapsSequenceInput(const asn1SccCorrespondenceMaps2DSequence& data)
{
    inCorrespondenceMapsSequence = data;
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
