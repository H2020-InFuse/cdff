/**
 * @addtogroup DFPCs
 * @{
 */

#include "DEMBuildingInterface.hpp"

namespace CDFF
{
namespace DFPC
{

DEMBuildingInterface::DEMBuildingInterface()
{
    asn1SccPointcloud_Initialize(& inLPC);
    asn1SccTransformWithCovariance_Initialize(& inEstimatedPose);
}

DEMBuildingInterface::~DEMBuildingInterface()
{
}

void DEMBuildingInterface::lPCInput(const asn1SccPointcloud& data)
{
    inLPC = data;
}

void DEMBuildingInterface::estimatedPoseInput(const asn1SccTransformWithCovariance& data)
{
    inEstimatedPose = data;
}

/*
*const asn1SccMap& DEMBuildingInterface::updatedMapOutput() const
*{
*    return outUpdatedMap;
*}
*/

}
}

/** @} */
