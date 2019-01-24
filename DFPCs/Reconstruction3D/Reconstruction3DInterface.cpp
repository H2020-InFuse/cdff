/**
 * @addtogroup DFPCs
 * @{
 */

#include "Reconstruction3DInterface.hpp"

namespace CDFF
{
namespace DFPC
{

Reconstruction3DInterface::Reconstruction3DInterface()
{
    asn1SccFrame_Initialize(& inLeftImage);
    asn1SccFrame_Initialize(& inRightImage);
    asn1SccPointcloud_Initialize(& outPointCloud);
    asn1SccPose_Initialize(& outPose);
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
}

/** @} */
