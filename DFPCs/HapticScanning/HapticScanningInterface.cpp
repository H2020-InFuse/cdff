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
void HapticScanningInterface::positionInput(const asn1SccPointSequence & positions)
{
    inPositions = positions;
}

//=====================================================================================================================
void HapticScanningInterface::forceInput(const asn1SccDoubleSequence & forces)
{
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
