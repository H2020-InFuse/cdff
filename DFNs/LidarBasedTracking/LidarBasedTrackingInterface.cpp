/**
 * @addtogroup DFNs
 * @{
 */

#include "LidarBasedTrackingInterface.hpp"

namespace CDFF
{
namespace DFN
{

LidarBasedTrackingInterface::LidarBasedTrackingInterface()
{
}

LidarBasedTrackingInterface::~LidarBasedTrackingInterface()
{
}

void LidarBasedTrackingInterface::sourceCloudInput(const asn1SccPointcloud& data)
{
    inSourceCloud = data;
}

const asn1SccRigidBodyState& LidarBasedTrackingInterface::stateOutput() const
{
    return outState;
}

}
}

/** @} */
