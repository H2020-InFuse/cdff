/**
 * @addtogroup DFNs
 * @{
 */

#include "ForceMeshGeneratorInterface.hpp"

namespace CDFF
{
namespace DFN
{

//=====================================================================================================================
ForceMeshGeneratorInterface::ForceMeshGeneratorInterface()
{
}

//=====================================================================================================================
void ForceMeshGeneratorInterface::roverPoseInput(const asn1SccPose& data)
{
    inRoverPose = data;
}

//=====================================================================================================================
void ForceMeshGeneratorInterface::positionAndForceInput(const asn1SccPointsSequence & positions, const asn1SccDoublesSequence & forces)
{
    inPositions = positions;
    inForces = forces;
}

//=====================================================================================================================
const asn1SccPointcloud& ForceMeshGeneratorInterface::pointCloudOutput() const
{
    return outPointCloud;
}

}
}

/** @} */
