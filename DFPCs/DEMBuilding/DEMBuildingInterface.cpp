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
