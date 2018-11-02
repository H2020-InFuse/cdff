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
namespace Executors
{

//=====================================================================================================================
void Execute(PrimitiveMatchingInterface* dfn, FrameConstPtr inputFrame, const asn1SccStringSequence& inputPrimitiveSequence, asn1SccStringSequence outputPrimitiveSequence)
{
	Execute(dfn, *inputFrame, inputPrimitiveSequence, outputPrimitiveSequence);
}

//=====================================================================================================================
void Execute(PrimitiveMatchingInterface* dfn, const Frame& inputFrame, const asn1SccStringSequence& inputPrimitiveSequence, asn1SccStringSequence outputPrimitiveSequence)
{
	ASSERT( dfn!= NULL, "PrimitiveMatchingExecutor, input dfn is null");
	dfn->imageInput(inputFrame);
	dfn->primitivesInput(inputPrimitiveSequence);
	dfn->process();

	outputPrimitiveSequence = dfn->primitivesOutput();
}

}
}
}

/** @} */
