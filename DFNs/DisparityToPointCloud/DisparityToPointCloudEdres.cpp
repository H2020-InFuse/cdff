/**
 * @addtogroup DFNs
 * @{
 */

#include "DisparityToPointCloudEdres.hpp"
#include "edres-wrapper/EdresStereo.h"

namespace CDFF
{
namespace DFN
{
namespace DisparityToPointCloud
{

DisparityToPointCloudEdres::DisparityToPointCloudEdres()
{
}

DisparityToPointCloudEdres::~DisparityToPointCloudEdres()
{
}

void DisparityToPointCloudEdres::configure()
{
}

void DisparityToPointCloudEdres::process()
{
    Edres::pointCloudReprojection(inDispImage, outPointCloud);
}

}
}
}

/** @} */
