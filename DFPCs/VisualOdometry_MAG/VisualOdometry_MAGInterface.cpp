/**
 * @addtogroup DFPCs
 * @{
 */

#include "VisualOdometry_MAGInterface.hpp"

namespace CDFF
{
namespace DFPC
{

VisualOdometry_MAGInterface::VisualOdometry_MAGInterface()
{
}

VisualOdometry_MAGInterface::~VisualOdometry_MAGInterface()
{
}

void VisualOdometry_MAGInterface::leftImageInput(const asn1SccFrame& data)
{
    inLeftImage = data;
}

void VisualOdometry_MAGInterface::rightImageInput(const asn1SccFrame& data)
{
    inRightImage = data;
}

void VisualOdometry_MAGInterface::odoMotionInput(const asn1SccTransformWithCovariance& data)
{
    inOdoMotion = data;
}

const asn1SccTransformWithCovariance& VisualOdometry_MAGInterface::estimateMotionOutput() const
{
    return outEstimateMotion;
}

}
}

/** @} */
