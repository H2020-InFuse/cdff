/**
 * @addtogroup DFPCs
 * @{
 */

#include "ReconstructionAndIdentificationInterface.hpp"

namespace CDFF
{
namespace DFPC
{

ReconstructionAndIdentificationInterface::ReconstructionAndIdentificationInterface() :
inLeftImage(),
inRightImage(),
inModel(),
inComputeModelFeatures(),
outPointCloud(),
outPose(),
outSuccess()
{
}

ReconstructionAndIdentificationInterface::~ReconstructionAndIdentificationInterface()
{
}

void ReconstructionAndIdentificationInterface::leftImageInput(const asn1SccFrame& data)
{
    inLeftImage = data;
}

void ReconstructionAndIdentificationInterface::rightImageInput(const asn1SccFrame& data)
{
    inRightImage = data;
}

void ReconstructionAndIdentificationInterface::modelInput(const asn1SccPointcloud& data)
{
    inModel = data;
}

void ReconstructionAndIdentificationInterface::computeModelFeaturesInput(bool data)
{
    inComputeModelFeatures = data;
}

const asn1SccPointcloud& ReconstructionAndIdentificationInterface::pointCloudOutput() const
{
    return outPointCloud;
}

const asn1SccPose& ReconstructionAndIdentificationInterface::poseOutput() const
{
    return outPose;
}

bool ReconstructionAndIdentificationInterface::successOutput() const
{
    return outSuccess;
}

}
}

/** @} */
