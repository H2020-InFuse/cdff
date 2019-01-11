/**
 * @addtogroup DFNs
 * @{
 */

#include "DisparityToPointCloudWithIntensityEdres.hpp"
#include "edres-wrapper/EdresStereo.h"

namespace CDFF
{
namespace DFN
{
namespace DisparityToPointCloudWithIntensity
{

DisparityToPointCloudWithIntensityEdres::DisparityToPointCloudWithIntensityEdres()
{
}

DisparityToPointCloudWithIntensityEdres::~DisparityToPointCloudWithIntensityEdres()
{
}

void DisparityToPointCloudWithIntensityEdres::configure()
{
}

void DisparityToPointCloudWithIntensityEdres::process()
{
    Edres::pointCloudReprojectionWithIntensity(inDispImage, inIntensityImage, outPointCloud);
}

}
}
}

/** @} */
