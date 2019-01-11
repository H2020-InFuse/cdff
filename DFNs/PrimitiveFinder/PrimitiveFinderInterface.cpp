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
