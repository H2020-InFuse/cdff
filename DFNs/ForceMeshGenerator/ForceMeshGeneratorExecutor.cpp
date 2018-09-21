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
void ForceMeshGeneratorExecutor::Execute(const asn1SccPose &roverPose, const asn1SccPointsSequence &positions, const asn1SccDoublesSequence &forces, PointCloudWrapper::PointCloudPtr outputPointCloud)
{
    Execute(roverPose, positions, forces, *outputPointCloud);
}

//=====================================================================================================================
void ForceMeshGeneratorExecutor::Execute(const asn1SccPose& roverPose, const asn1SccPointsSequence & positions, const asn1SccDoublesSequence & forces, PointCloudWrapper::PointCloud & outputPointCloud)
{
    dfn->roverPoseInput(roverPose);
    dfn->positionInput(positions);
    dfn->forceInput(forces);
    dfn->process();
    const asn1SccPointcloud& output = dfn->pointCloudOutput();
    PointCloudWrapper::Copy(output, outputPointCloud);
}

}
}

/** @} */
