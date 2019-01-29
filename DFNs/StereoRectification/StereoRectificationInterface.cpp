/**
 * @addtogroup DFNs
 * @{
 */

#include "StereoRectificationInterface.hpp"

namespace CDFF
{
namespace DFN
{

StereoRectificationInterface::StereoRectificationInterface()
:              inOriginalStereoPair(),
               outRectifiedStereoPair()
{
    asn1SccFramePair_Initialize(& inOriginalStereoPair);
    asn1SccFramePair_Initialize(& outRectifiedStereoPair);
}

StereoRectificationInterface::~StereoRectificationInterface()
{
}

void StereoRectificationInterface::originalStereoPairInput(const asn1SccFramePair& data)
{
    inOriginalStereoPair = data;
}

const asn1SccFramePair& StereoRectificationInterface::rectifiedStereoPairOutput() const
{
    return outRectifiedStereoPair;
}

}
}

/** @} */
