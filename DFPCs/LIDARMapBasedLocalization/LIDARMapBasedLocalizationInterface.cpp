/**
 * @addtogroup DFPCs
 * @{
 */

#include "LIDARMapBasedLocalizationInterface.hpp"

namespace CDFF
{
namespace DFPC
{

LIDARMapBasedLocalizationInterface::LIDARMapBasedLocalizationInterface()
{
    asn1SccPointcloud_Initialize(& inLPC);
    asn1SccTransformWithCovariance_Initialize(& inOdoPose);
    asn1SccTransformWithCovariance_Initialize(& outLidarPose);
}

LIDARMapBasedLocalizationInterface::~LIDARMapBasedLocalizationInterface()
{
}

void LIDARMapBasedLocalizationInterface::lPCInput(const asn1SccPointcloud& data)
{
    inLPC = data;
}

void LIDARMapBasedLocalizationInterface::odoPoseInput(const asn1SccTransformWithCovariance& data)
{
    inOdoPose = data;
}

const asn1SccTransformWithCovariance& LIDARMapBasedLocalizationInterface::lidarPoseOutput() const
{
    return outLidarPose;
}

}
}

/** @} */
