/**
 * @addtogroup DFNs
 * @{
 */

#include "ModelBasedDetectionInterface.hpp"

namespace CDFF
{
namespace DFN
{

ModelBasedDetectionInterface::ModelBasedDetectionInterface() : outSuccess(false)
{
    asn1SccFrame_Initialize(&inimage);
    asn1SccFrame_Initialize(&indepth);
    asn1SccPose_Initialize(&outPose);
    asn1SccMatrix2d_Initialize(&outDetectionBoundingBox);
}

ModelBasedDetectionInterface::~ModelBasedDetectionInterface()
{
}

void ModelBasedDetectionInterface::imageInput(asn1SccFrame &data)
{
    inimage = data;
}

void ModelBasedDetectionInterface::depthInput(asn1SccFrame& data) {
    indepth = data;
}

const asn1SccPose& ModelBasedDetectionInterface::poseOutput() const
{
    return outPose;
}

const asn1SccMatrix2d& ModelBasedDetectionInterface::detectionBoundingBoxOutput() const
{
    return outDetectionBoundingBox;
}

bool ModelBasedDetectionInterface::successOutput() const
{
    return outSuccess;
}

}
}

/** @} */
