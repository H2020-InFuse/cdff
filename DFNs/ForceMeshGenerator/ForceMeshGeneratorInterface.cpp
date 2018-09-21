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
void ForceMeshGeneratorInterface::roverPoseInput(const asn1SccPose& pose)
{
    inRoverPose = pose;
}

//=====================================================================================================================
void ForceMeshGeneratorInterface::positionInput(const asn1SccPointsSequence & positions)
{
    inPositions = positions;
}

//=====================================================================================================================
void ForceMeshGeneratorInterface::forceInput(const asn1SccDoublesSequence & forces)
{
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
