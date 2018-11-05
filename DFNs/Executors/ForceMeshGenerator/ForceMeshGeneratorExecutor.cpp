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
namespace Executors
{

//=====================================================================================================================
void Execute(
    ForceMeshGeneratorInterface* dfn,
    const asn1SccPose &armBasePose,
    const asn1SccPose &armEndEffectorPose,
    const asn1SccWrench &armEndEffectorWrench,
    PointCloudWrapper::PointCloudPtr outputPointCloud
    )
{
    Execute(dfn, armBasePose, armEndEffectorPose, armEndEffectorWrench, outputPointCloud);
}

//=====================================================================================================================
void Execute(
    ForceMeshGeneratorInterface* dfn,
    const asn1SccPose &armBasePose,
    const asn1SccPose &armEndEffectorPose,
    const asn1SccWrench &armEndEffectorWrench,
    PointCloudWrapper::PointCloud & outputPointCloud
    )
{
    ASSERT( dfn!= NULL, "ForceMeshGeneratorExecutor, input dfn is null");
    dfn->armBasePoseInput(armBasePose);
    dfn->armEndEffectorPoseInput(armEndEffectorPose);
    dfn->armEndEffectorWrenchInput(armEndEffectorWrench);
    dfn->process();

    const asn1SccPointcloud& output = dfn->pointCloudOutput();
    PointCloudWrapper::Copy(output, outputPointCloud);
}

}
}
}

/** @} */
