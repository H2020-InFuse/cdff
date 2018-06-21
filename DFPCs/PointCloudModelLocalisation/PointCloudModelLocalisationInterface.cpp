/**
 * @addtogroup DFPCs
 * @{
 */

#include "PointCloudModelLocalisationInterface.hpp"

namespace dfpc_ci
{

PointCloudModelLocalisationInterface::PointCloudModelLocalisationInterface()
{
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

/** @} */
