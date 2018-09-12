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
asn1SccT_String PrimitiveMatchingInterface::primitiveMatchedOutput() const
{
    return outPrimitiveMatched;
}

//=====================================================================================================================
const asn1SccFrame& PrimitiveMatchingInterface::imageWithMatchedContourOutput() const
{
    return outImageWithMatchedContour;
}

}
}

/** @} */
