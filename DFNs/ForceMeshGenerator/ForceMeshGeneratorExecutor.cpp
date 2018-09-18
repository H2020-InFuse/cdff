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
void ForceMeshGeneratorExecutor::Execute(const asn1SccPose &roverPose, const asn1SccPointArray &positions, const asn1SccDoubleArray &forces, PointCloudWrapper::PointCloudPtr outputPointCloud)
{
    Execute(roverPose, positions, forces, *outputPointCloud);
}

//=====================================================================================================================
void ForceMeshGeneratorExecutor::Execute(const asn1SccPose& roverPose, const asn1SccPointArray & positions, const asn1SccDoubleArray & forces, PointCloudWrapper::PointCloud & outputPointCloud)
{
    dfn->roverPoseInput(roverPose);
    dfn->positionAndForceInput(positions, forces);
    dfn->process();
    const asn1SccPointcloud& output = dfn->pointCloudOutput();
    PointCloudWrapper::Copy(output, outputPointCloud);
}

}
}

/** @} */
