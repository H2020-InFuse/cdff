/**
 * @addtogroup DFNs
 * @{
 */

#include "BundleAdjustmentInterface.hpp"

namespace CDFF
{
namespace DFN
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

void BundleAdjustmentInterface::guessedPosesSequenceInput(const asn1SccPosesSequence& data)
{
inGuessedPosesSequence = data;
}

void BundleAdjustmentInterface::guessedPointCloudInput(const asn1SccPointcloud& data)
{
inGuessedPointCloud = data;
}

const asn1SccPosesSequence& BundleAdjustmentInterface::posesSequenceOutput() const
{
    return outPosesSequence;
}

bool BundleAdjustmentInterface::successOutput() const
{
    return outSuccess;
}

float BundleAdjustmentInterface::errorOutput() const
{
    return outError;
}

}
}

/** @} */
