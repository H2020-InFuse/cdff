/**
 * @addtogroup DFPCs
 * @{
 */

#include "ModelBasedTrackerInterface.hpp"

namespace CDFF
{
namespace DFPC
{

ModelBasedTrackerInterface::ModelBasedTrackerInterface()
{
    asn1SccFrame_Initialize(& inImage);
    asn1SccFrame_Initialize(& inDepth);
    asn1SccT_String_Initialize(& inRobotName);
    asn1SccPose_Initialize(& outPose);
}

ModelBasedTrackerInterface::~ModelBasedTrackerInterface()
{
}

void ModelBasedTrackerInterface::imageInput(const asn1SccFrame& data)
{
    inImage = data;
}

void ModelBasedTrackerInterface::depthInput(const asn1SccFrame& data)
{
    inDepth = data;
}

void ModelBasedTrackerInterface::robotNameInput(const asn1SccT_String& data)
{
    inRobotName = data;
}

const asn1SccPose& ModelBasedTrackerInterface::poseOutput() const
{
    return outPose;
}

}
}

/** @} */
