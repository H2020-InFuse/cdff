/**
 * @addtogroup DFPCs
 * @{
 */

#include "Reconstruction3DInterface.hpp"

namespace dfpc_ci
{

Reconstruction3DInterface::Reconstruction3DInterface()
{
}

Reconstruction3DInterface::~Reconstruction3DInterface()
{
}

void Reconstruction3DInterface::leftImageInput(const asn1SccFrame& data)
{
    inLeftImage = data;
}

void Reconstruction3DInterface::rightImageInput(const asn1SccFrame& data)
{
    inRightImage = data;
}

const asn1SccPointcloud& Reconstruction3DInterface::pointCloudOutput() const
{
    return outPointCloud;
}

const asn1SccPose& Reconstruction3DInterface::poseOutput() const
{
    return outPose;
}

bool Reconstruction3DInterface::successOutput() const
{
    return outSuccess;
}

}

/** @} */
