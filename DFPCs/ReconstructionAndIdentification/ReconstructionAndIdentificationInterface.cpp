/**
 * @addtogroup DFPCs
 * @{
 */

#include "ReconstructionAndIdentificationInterface.hpp"

namespace CDFF
{
namespace DFPC
{

ReconstructionAndIdentificationInterface::ReconstructionAndIdentificationInterface()
{
    asn1SccFrame_Initialize(& inLeftImage);
    asn1SccFrame_Initialize(& inRightImage);
    asn1SccPointcloud_Initialize(& inModel);

    asn1SccPointcloud_Initialize(& outPointCloud);
    asn1SccPose_Initialize(& outPose);
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
