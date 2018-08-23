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
