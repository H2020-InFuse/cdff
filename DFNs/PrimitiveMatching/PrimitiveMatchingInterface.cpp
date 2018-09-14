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
void PrimitiveMatchingInterface::primitiveArrayInput(const BaseTypesWrapper::asn1SccT_StringArray& data)
{
    inPrimitiveArray = data;
}

//=====================================================================================================================
BaseTypesWrapper::asn1SccT_StringArray PrimitiveMatchingInterface::primitivesMatchedOutput() const
{
    return outPrimitiveArrayMatched;
}

//=====================================================================================================================
const asn1SccFrame& PrimitiveMatchingInterface::imageWithMatchedContourOutput() const
{
    return outImageWithMatchedContour;
}

}
}

/** @} */
