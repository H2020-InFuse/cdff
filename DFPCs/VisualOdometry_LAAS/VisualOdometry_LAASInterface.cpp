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
    asn1SccFrame_Initialize(& inLeftImage);
    asn1SccFrame_Initialize(& inRightImage);
    asn1SccTransformWithCovariance_Initialize(& outEstimatedMotion);
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
