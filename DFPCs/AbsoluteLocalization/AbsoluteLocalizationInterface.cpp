/**
 * @addtogroup DFPCs
 * @{
 */

#include "AbsoluteLocalizationInterface.hpp"

namespace CDFF
{
namespace DFPC
{

AbsoluteLocalizationInterface::AbsoluteLocalizationInterface()
{
    asn1SccPointcloud_Initialize(& inLidarPC);
    asn1SccTransformWithCovariance_Initialize(& inEstimatedPose);
    asn1SccFrame_Initialize(& inOrthoImage);
    asn1SccFrame_Initialize(& inOrbitalImage);
    asn1SccTransformWithCovariance_Initialize(& outFinalPose);
}

AbsoluteLocalizationInterface::~AbsoluteLocalizationInterface()
{
}

void AbsoluteLocalizationInterface::lidarPCInput(const asn1SccPointcloud& data)
{
    inLidarPC = data;
}

void AbsoluteLocalizationInterface::estimatedPoseInput(const asn1SccTransformWithCovariance& data)
{
    inEstimatedPose = data;
}

void AbsoluteLocalizationInterface::orthoImageInput(const asn1SccFrame& data)
{
    inOrthoImage = data;
}

void AbsoluteLocalizationInterface::orbitalImageInput(const asn1SccFrame& data)
{
    inOrbitalImage = data;
}

const asn1SccTransformWithCovariance& AbsoluteLocalizationInterface::finalPoseOutput() const
{
    return outFinalPose;
}

}

}

/** @} */
