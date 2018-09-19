/**
 * @addtogroup DFPCs
 * @{
 */

#include "HapticScanningInterface.hpp"

namespace CDFF
{
namespace DFPC
{

//=====================================================================================================================
HapticScanningInterface::HapticScanningInterface()
{
}

//=====================================================================================================================
HapticScanningInterface::~HapticScanningInterface()
{
}

//=====================================================================================================================
void HapticScanningInterface::roverPoseInput(const asn1SccPose& data)
{
    inRoverPose = data;
}

//=====================================================================================================================
void HapticScanningInterface::positionAndForceInput(const asn1SccPointsSequence & positions, const asn1SccDoublesSequence & forces)
{
    inPositions = positions;
    inForces = forces;
}

//=====================================================================================================================
const asn1SccPointcloud& HapticScanningInterface::pointCloudOutput() const
{
    return outPointCloud;
}


}

}

/** @} */
