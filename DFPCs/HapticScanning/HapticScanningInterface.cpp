/**
 * @addtogroup DFPCs
 * @{
 */

#include "HapticScanningInterface.hpp"
//#include <Types/CPP/PointCloud.hpp>

namespace CDFF
{
namespace DFPC
{

//=====================================================================================================================
HapticScanningInterface::HapticScanningInterface() :
inArmBasePose(),
inArmEndEffectorPose(),
inArmEndEffectorWrench()
{
    asn1SccPointcloud_Initialize(&outPointCloud);
}

//=====================================================================================================================
HapticScanningInterface::~HapticScanningInterface()
{
}

//=====================================================================================================================
void HapticScanningInterface::armBasePoseInput(const asn1SccPose& data)
{
    inArmBasePose = data;
}


//=====================================================================================================================
void HapticScanningInterface::armEndEffectorPoseInput(const asn1SccPose &pose)
{
    inArmEndEffectorPose = pose;
}

//=====================================================================================================================
void HapticScanningInterface::armEndEffectorWrenchInput(const asn1SccWrench &wrench)
{
    inArmEndEffectorWrench = wrench;
}

//=====================================================================================================================
const asn1SccPointcloud &HapticScanningInterface::pointCloudOutput() const
{
    return outPointCloud;
}


}

}

/** @} */
