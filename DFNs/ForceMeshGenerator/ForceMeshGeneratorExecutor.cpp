/**
 * @addtogroup DFNs
 * @{
 */

#include "ForceMeshGeneratorExecutor.hpp"
#include <Errors/Assert.hpp>

namespace CDFF
{
namespace DFN
{

//=====================================================================================================================
ForceMeshGeneratorExecutor::ForceMeshGeneratorExecutor(ForceMeshGeneratorInterface* dfn)
    : dfn (dfn)
{
    ASSERT(dfn != NULL, "ForceMeshGeneratorExecutor: null dfn in input");
}

//=====================================================================================================================
void ForceMeshGeneratorExecutor::Execute(
    const asn1SccPose &armBasePose,
    const asn1SccPose &armEndEffectorPose,
    const asn1SccWrench &armEndEffectorWrench,
    PointCloudWrapper::PointCloudPtr outputPointCloud
    )
{
    Execute(armBasePose, armEndEffectorPose, armEndEffectorWrench, *outputPointCloud);
}

//=====================================================================================================================
void ForceMeshGeneratorExecutor::Execute(
    const asn1SccPose &armBasePose,
    const asn1SccPose &armEndEffectorPose,
    const asn1SccWrench &armEndEffectorWrench,
    PointCloudWrapper::PointCloud & outputPointCloud
    )
{
    dfn->armBasePoseInput(armBasePose);
    dfn->armEndEffectorPoseInput(armEndEffectorPose);
    dfn->armEndEffectorWrenchInput(armEndEffectorWrench);
    dfn->process();

    const asn1SccPointcloud& output = dfn->pointCloudOutput();
    PointCloudWrapper::Copy(output, outputPointCloud);
}

}
}

/** @} */
