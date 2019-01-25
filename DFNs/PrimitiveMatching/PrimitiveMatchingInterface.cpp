/**
 * @addtogroup DFNs
 * @{
 */

#include "PrimitiveMatchingInterface.hpp"

namespace CDFF
{
namespace DFN
{

PrimitiveMatchingInterface::PrimitiveMatchingInterface()
{
    asn1SccFrame_Initialize(& inImage);
    asn1SccStringSequence_Initialize(& inPrimitives);
    asn1SccFrame_Initialize(& outImage);
    asn1SccStringSequence_Initialize(& outPrimitives);
}

PrimitiveMatchingInterface::~PrimitiveMatchingInterface()
{
}

void PrimitiveMatchingInterface::imageInput(const asn1SccFrame& data)
{
    inImage = data;
}

void PrimitiveMatchingInterface::primitivesInput(const asn1SccStringSequence& data)
{
    inPrimitives = data;
}

const asn1SccFrame& PrimitiveMatchingInterface::imageOutput() const
{
    return outImage;
}

const asn1SccStringSequence& PrimitiveMatchingInterface::primitivesOutput() const
{
    return outPrimitives;
}

}
}

/** @} */
