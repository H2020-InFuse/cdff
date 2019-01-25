/**
 * @addtogroup DFNs
 * @{
 */

#include "LinemodDetectInterface.hpp"

namespace CDFF
{
namespace DFN
{

LinemodDetectInterface::LinemodDetectInterface()
{
    asn1SccFrame_Initialize(& inimage);
    asn1SccFrame_Initialize(& indepth);
    asn1SccPose_Initialize(& outCamera);
}

LinemodDetectInterface::~LinemodDetectInterface()
{
}

void LinemodDetectInterface::imageInput(asn1SccFrame &data)
{
    inimage = data;
}

void LinemodDetectInterface::depthInput(asn1SccFrame& data) {
    indepth = data;
}

const asn1SccPose& LinemodDetectInterface::cameraOutput() const
{
    return outCamera;
}

bool LinemodDetectInterface::successOutput() const
{
    return outSuccess;
}

}
}

/** @} */
