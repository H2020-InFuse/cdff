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
    asn1SccCorrespondenceMaps2DSequence_Initialize(& inCorrespondenceMapsSequence);
    asn1SccPosesSequence_Initialize(& inGuessedPosesSequence);
    asn1SccPointcloud_Initialize(& inGuessedPointCloud);
    asn1SccPosesSequence_Initialize(& outPosesSequence);
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
