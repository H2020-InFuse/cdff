/**
 * @addtogroup DFNs
 * @{
 */

#include "VoxelizationExecutor.hpp"
#include <Errors/Assert.hpp>

namespace CDFF
{
namespace DFN
{
namespace Executors
{

//=====================================================================================================================
void Execute(VoxelizationInterface* dfn, FrameWrapper::FrameConstPtr inputFrame, asn1SccOctree & outputOctree)
{
    Execute(dfn, *inputFrame, outputOctree);
}

//=====================================================================================================================
void Execute(VoxelizationInterface* dfn, FrameWrapper::FrameConstPtr inputFrame, const asn1SccOctree * outputOctree)
{
    ASSERT(outputOctree != NULL, "VoxelizationExecutor, Calling NO instance creation Executor with a NULL pointer");
    Execute(dfn, *inputFrame, outputOctree);
}

//=====================================================================================================================
void Execute(VoxelizationInterface* dfn, const FrameWrapper::Frame& inputFrame, const asn1SccOctree * outputOctree)
{
    ASSERT( dfn!= NULL, "VoxelizationExecutor, input dfn is null");
    ASSERT( outputOctree == NULL, "VoxelizationExecutor, Calling instance creation executor with a non-NULL pointer");
    dfn->depthInput(inputFrame);
    dfn->process();
    outputOctree = & ( dfn->octreeOutput() );
}

//=====================================================================================================================
void Execute(VoxelizationInterface* dfn, const FrameWrapper::Frame& inputFrame, asn1SccOctree& outputOctree)
{
    ASSERT( dfn!= NULL, "VoxelizationExecutor, input dfn is null");
    dfn->depthInput(inputFrame);
    dfn->process();
    outputOctree = dfn->octreeOutput();
}

}
}
}

/** @} */
