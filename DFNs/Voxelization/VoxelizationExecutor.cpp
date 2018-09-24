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

//=====================================================================================================================
VoxelizationExecutor::VoxelizationExecutor(VoxelizationInterface* dfn)
    : dfn (dfn)
{
    ASSERT(dfn != NULL, "VoxelizationExecutor: null dfn in input");
}

//=====================================================================================================================
void VoxelizationExecutor::Execute(FrameWrapper::FrameConstPtr inputFrame, asn1SccOctree & outputOctree)
{
    Execute(*inputFrame, outputOctree);
}

//=====================================================================================================================
void VoxelizationExecutor::Execute(FrameWrapper::FrameConstPtr inputFrame, const asn1SccOctree * outputOctree)
{
    ASSERT(outputOctree != NULL, "VoxelizationExecutor, Calling NO instance creation Executor with a NULL pointer");
    Execute(*inputFrame, outputOctree);
}

//=====================================================================================================================
void VoxelizationExecutor::Execute(const FrameWrapper::Frame& inputFrame, const asn1SccOctree * outputOctree)
{
    ASSERT( outputOctree == NULL, "VoxelizationExecutor, Calling instance creation executor with a non-NULL pointer");
    dfn->frameInput(inputFrame);
    dfn->process();
    outputOctree = & ( dfn->octreeOutput() );
}

//=====================================================================================================================
void VoxelizationExecutor::Execute(const FrameWrapper::Frame& inputFrame, asn1SccOctree& outputOctree)
{
    dfn->frameInput(inputFrame);
    dfn->process();
    outputOctree = dfn->octreeOutput();
}

}
}

/** @} */
