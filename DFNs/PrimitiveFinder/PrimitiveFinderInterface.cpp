/**
 * @addtogroup DFNs
 * @{
 */

#include "PrimitiveFinderInterface.hpp"

namespace CDFF
{
namespace DFN
{

PrimitiveFinderInterface::PrimitiveFinderInterface()
{
    asn1SccFrame_Initialize(& inImage);
    asn1SccT_String_Initialize(& inPrimitive);
    asn1SccVectorXdSequence_Initialize(& outPrimitives);
}

PrimitiveFinderInterface::~PrimitiveFinderInterface()
{
}

void PrimitiveFinderInterface::imageInput(const asn1SccFrame& data)
{
    inImage = data;
}

void PrimitiveFinderInterface::primitiveInput(const asn1SccT_String& data)
{
    inPrimitive = data;
}

const asn1SccVectorXdSequence& PrimitiveFinderInterface::primitivesOutput() const
{
    return outPrimitives;
}

}
}

/** @} */
