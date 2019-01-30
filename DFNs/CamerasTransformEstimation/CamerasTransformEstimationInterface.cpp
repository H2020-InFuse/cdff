/**
 * @addtogroup DFNs
 * @{
 */

#include "CamerasTransformEstimationInterface.hpp"

namespace CDFF
{
namespace DFN
{

CamerasTransformEstimationInterface::CamerasTransformEstimationInterface()
{
    asn1SccMatrix3d_Initialize(&inFundamentalMatrix);
    asn1SccCorrespondenceMap2D_Initialize(&inMatches);
    asn1SccPose_Initialize(&outTransform);
}

CamerasTransformEstimationInterface::~CamerasTransformEstimationInterface()
{
}

void CamerasTransformEstimationInterface::fundamentalMatrixInput(const asn1SccMatrix3d& data)
{
    inFundamentalMatrix = data;
}

void CamerasTransformEstimationInterface::matchesInput(const asn1SccCorrespondenceMap2D& data)
{
    inMatches = data;
}

const asn1SccPose& CamerasTransformEstimationInterface::transformOutput() const
{
    return outTransform;
}

bool CamerasTransformEstimationInterface::successOutput() const
{
    return outSuccess;
}

}
}

/** @} */
