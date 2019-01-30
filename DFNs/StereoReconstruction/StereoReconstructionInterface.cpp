/**
 * @addtogroup DFNs
 * @{
 */

#include "StereoReconstructionInterface.hpp"

namespace CDFF
{
namespace DFN
{

StereoReconstructionInterface::StereoReconstructionInterface()
{
    asn1SccFrame_Initialize(& inLeft);
    asn1SccFrame_Initialize(& inRight);
    asn1SccPointcloud_Initialize(& outPointcloud);
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
}

/** @} */
