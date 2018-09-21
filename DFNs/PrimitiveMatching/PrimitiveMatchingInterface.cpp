/**
 * @addtogroup DFNs
 * @{
 */

#include "PrimitiveMatchingInterface.hpp"

namespace CDFF
{
namespace DFN
{

//=====================================================================================================================
PrimitiveMatchingInterface::PrimitiveMatchingInterface()
{
}

//=====================================================================================================================
PrimitiveMatchingInterface::~PrimitiveMatchingInterface()
{
}

//=====================================================================================================================
void PrimitiveMatchingInterface::frameInput(const asn1SccFrame& data)
{
    inImage = data;
}

//=====================================================================================================================
void PrimitiveMatchingInterface::primitiveSequenceInput(const asn1SccStringSequence& data)
{
    inPrimitiveSequence = data;
}

//=====================================================================================================================
const asn1SccStringSequence & PrimitiveMatchingInterface::primitivesMatchedOutput() const
{
    return outPrimitiveSequenceMatched;
}

//=====================================================================================================================
const asn1SccFrame& PrimitiveMatchingInterface::imageWithMatchedContourOutput() const
{
    return outImageWithMatchedContour;
}

}
}

/** @} */
