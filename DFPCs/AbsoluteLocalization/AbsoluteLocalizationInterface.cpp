/**
 * @addtogroup DFPCs
 * @{
 */

#include "AbsoluteLocalizationInterface.hpp"

namespace dfpc_ci
{

AbsoluteLocalizationInterface::AbsoluteLocalizationInterface()
{
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

/** @} */
