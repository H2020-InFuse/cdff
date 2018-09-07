/**
 * @addtogroup DFNs
 * @{
 */

#include "PrimitiveMatchingExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace FrameWrapper;

namespace CDFF
{
namespace DFN
{

//=====================================================================================================================
PrimitiveMatchingExecutor::PrimitiveMatchingExecutor(PrimitiveMatchingInterface* dfn)
{
	this->dfn = dfn;
}

//=====================================================================================================================
PrimitiveMatchingExecutor::~PrimitiveMatchingExecutor()
{

}

//=====================================================================================================================
void PrimitiveMatchingExecutor::Execute(FrameConstPtr inputFrame, asn1SccT_String outputPrimitive)
{
	Execute(inputFrame, outputPrimitive);
}

//=====================================================================================================================
void PrimitiveMatchingExecutor::Execute(const Frame& inputFrame, asn1SccT_String outputPrimitive)
{
	dfn->frameInput(inputFrame);
	dfn->process();
	BaseTypesWrapper::CopyString( dfn->primitiveMatchedOutput(), outputPrimitive);
}

}
}

/** @} */
