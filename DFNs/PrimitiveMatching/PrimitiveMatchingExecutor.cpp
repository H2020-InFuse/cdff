/**
 * @addtogroup DFNs
 * @{
 */

#include "PrimitiveMatchingExecutor.hpp"
#include <Errors/Assert.hpp>

using namespace FrameWrapper;
using namespace BaseTypesWrapper;

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
void PrimitiveMatchingExecutor::Execute(FrameConstPtr inputFrame, const asn1SccT_StringArray& inputPrimitiveArray, asn1SccT_StringArray outputPrimitiveArray)
{
	Execute(inputFrame, inputPrimitiveArray, outputPrimitiveArray);
}

//=====================================================================================================================
void PrimitiveMatchingExecutor::Execute(const Frame& inputFrame, const asn1SccT_StringArray& inputPrimitiveArray, asn1SccT_StringArray outputPrimitiveArray) //TODO
{
	dfn->frameInput(inputFrame);
	dfn->primitiveArrayInput(inputPrimitiveArray);
	dfn->process();

	outputPrimitiveArray = dfn->primitivesMatchedOutput();
}

}
}

/** @} */
