/**
 * @addtogroup DFPCs
 * @{
 */

#include "PointCloudModelLocalisationInterface.hpp"

namespace CDFF
{
namespace DFPC
{

PointCloudModelLocalisationInterface::PointCloudModelLocalisationInterface()
{
    asn1SccPointcloud_Initialize(& inScene);
    asn1SccPointcloud_Initialize(& inModel);
    asn1SccPose_Initialize(& outPose);
}

PointCloudModelLocalisationInterface::~PointCloudModelLocalisationInterface()
{
}

void PointCloudModelLocalisationInterface::sceneInput(const asn1SccPointcloud& data)
{
    inScene = data;
}

void PointCloudModelLocalisationInterface::modelInput(const asn1SccPointcloud& data)
{
    inModel = data;
}

void PointCloudModelLocalisationInterface::computeModelFeaturesInput(bool data)
{
    inComputeModelFeatures = data;
}

const asn1SccPose& PointCloudModelLocalisationInterface::poseOutput() const
{
    return outPose;
}

bool PointCloudModelLocalisationInterface::successOutput() const
{
    return outSuccess;
}

}
}

/** @} */
