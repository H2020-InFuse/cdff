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
{
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
