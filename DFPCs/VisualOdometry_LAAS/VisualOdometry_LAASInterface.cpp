/**
 * @addtogroup DFPCs
 * @{
 */

#include "VisualOdometry_LAASInterface.hpp"

namespace CDFF
{
namespace DFPC
{

VisualOdometry_LAASInterface::VisualOdometry_LAASInterface() :
inLeftImage(),
inRightImage(),
outEstimatedMotion()
{
}

VisualOdometry_LAASInterface::~VisualOdometry_LAASInterface()
{
}

void VisualOdometry_LAASInterface::leftImageInput(const asn1SccFrame& data)
{
    inLeftImage = data;
}

void VisualOdometry_LAASInterface::rightImageInput(const asn1SccFrame& data)
{
    inRightImage = data;
}

const asn1SccTransformWithCovariance& VisualOdometry_LAASInterface::estimatedMotionOutput() const
{
    return outEstimatedMotion;
}

}
}
/** @} */
