/**
 * @addtogroup DFNs
 * @{
 */

#include "StereoReconstructionInterface.hpp"

namespace dfn_ci
{

StereoReconstructionInterface::StereoReconstructionInterface()
{
}

StereoReconstructionInterface::~StereoReconstructionInterface()
{
}

void StereoReconstructionInterface::leftInput(const asn1SccFrame& data)
{
    inLeft = data;
}

void StereoReconstructionInterface::rightInput(const asn1SccFrame& data)
{
    inRight = data;
}

const asn1SccPointcloud& StereoReconstructionInterface::pointcloudOutput() const
{
    return outPointcloud;
}

}

/** @} */
