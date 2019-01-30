/**
 * @addtogroup DFNs
 * @{
 */

#include "PointCloudAssemblyInterface.hpp"

namespace CDFF
{
namespace DFN
{

PointCloudAssemblyInterface::PointCloudAssemblyInterface()
{
    asn1SccPointcloud_Initialize(& inFirstPointCloud);
    asn1SccPointcloud_Initialize(& inSecondPointCloud);
    asn1SccPose_Initialize(& inViewCenter);

    asn1SccPointcloud_Initialize(& outAssembledPointCloud);
}

PointCloudAssemblyInterface::~PointCloudAssemblyInterface()
{
}

void PointCloudAssemblyInterface::firstPointCloudInput(const asn1SccPointcloud& data)
{
    inFirstPointCloud = data;
}

void PointCloudAssemblyInterface::secondPointCloudInput(const asn1SccPointcloud& data)
{
    inSecondPointCloud = data;
}

void PointCloudAssemblyInterface::viewCenterInput(const asn1SccPose& data)
{
    inViewCenter = data;
}

void PointCloudAssemblyInterface::viewRadiusInput(const float& data)
{
    inViewRadius = data;
}

const asn1SccPointcloud& PointCloudAssemblyInterface::assembledCloudOutput() const
{
    return outAssembledPointCloud;
}

}
}

/** @} */
