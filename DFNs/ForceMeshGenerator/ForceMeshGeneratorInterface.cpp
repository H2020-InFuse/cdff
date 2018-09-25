/**
 * @addtogroup DFNs
 * @{
 */

#include <Types/C/Wrench.h>
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
void ForceMeshGeneratorInterface::armBasePoseInput(const asn1SccPose &pose)
{
    inArmBasePose = pose;
}

//=====================================================================================================================
void ForceMeshGeneratorInterface::armEndEffectorPoseInput(const asn1SccPose &pose)
{
    inArmEndEffectorPose = pose;
}

//=====================================================================================================================
void ForceMeshGeneratorInterface::armEndEffectorWrenchInput(const asn1SccWrench wrench)
{
    inArmEndEffectorWrench = wrench;
}

//=====================================================================================================================
const asn1SccPointcloud& ForceMeshGeneratorInterface::pointCloudOutput() const
{
    return *outPointCloud;
}

}
}

/** @} */
