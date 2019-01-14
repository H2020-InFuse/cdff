/**
 * @addtogroup DFPCs
 * @{
 */

#include "MidRange3DModelTrackingInterface.hpp"

namespace CDFF
{
namespace DFPC
{

MidRange3DModelTrackingInterface::MidRange3DModelTrackingInterface()
{
}

MidRange3DModelTrackingInterface::~MidRange3DModelTrackingInterface()
{
}

void MidRange3DModelTrackingInterface::imageInput(const asn1SccFrame& data)
{
    inImage = data;
}

void MidRange3DModelTrackingInterface::depthInput(const asn1SccFrame& data)
{
    inDepth = data;
}

void MidRange3DModelTrackingInterface::robotNameInput(const asn1SccT_String& data)
{
    inRobotName = data;
}

const asn1SccPose& MidRange3DModelTrackingInterface::poseOutput() const
{
    return outPose;
}

}
}

/** @} */
