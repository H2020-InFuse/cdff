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
void PrimitiveMatchingExecutor::Execute(FrameConstPtr inputFrame, const asn1SccStringSequence& inputPrimitiveSequence, asn1SccStringSequence outputPrimitiveSequence)
{
	Execute(*inputFrame, inputPrimitiveSequence, outputPrimitiveSequence);
}

//=====================================================================================================================
void PrimitiveMatchingExecutor::Execute(const Frame& inputFrame, const asn1SccStringSequence& inputPrimitiveSequence, asn1SccStringSequence outputPrimitiveSequence)
{
	dfn->imageInput(inputFrame);
	dfn->primitivesInput(inputPrimitiveSequence);
	dfn->process();

	outputPrimitiveSequence = dfn->primitivesOutput();
}

}
}

/** @} */
