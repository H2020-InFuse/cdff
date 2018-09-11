/**
 * @addtogroup DFNs
 * @{
 */

#include "ForceMeshGeneratorInterface.hpp"

namespace CDFF
{
namespace DFN
{

//=====================================================================================================================
ForceMeshGeneratorInterface::ForceMeshGeneratorInterface()
{
}

//=====================================================================================================================
void ForceMeshGeneratorInterface::roverPoseInput(const asn1SccPose& data)
{
    inPose = data;
}

//=====================================================================================================================
void ForceMeshGeneratorInterface::positionAndForceInput(const asn1SccPosition & position, const asn1SccT_Double & force)
{
    inPosition = position;
    inForce = force;

    pcl::PointXYZ point (position.arr[0], position.arr[1], position.arr[2]);
    inPoints.push_back(std::make_pair(point, force));
}

//=====================================================================================================================
const asn1SccPointcloud& ForceMeshGeneratorInterface::pointCloudOutput() const
{
    return outPointCloud;
}

}
}

/** @} */
