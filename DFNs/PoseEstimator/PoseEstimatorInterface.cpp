/**
 * @addtogroup DFNs
 * @{
 */

#include "PoseEstimatorInterface.hpp"

namespace CDFF
{
namespace DFN
{

PoseEstimatorInterface::PoseEstimatorInterface()
{
    asn1SccFrame_Initialize(& inImage);
    asn1SccFrame_Initialize(& inDepth);
    asn1SccVectorXdSequence_Initialize(& inPrimitives);
    asn1SccPosesSequence_Initialize(& outPoses);
}

PoseEstimatorInterface::~PoseEstimatorInterface()
{
}

void PoseEstimatorInterface::imageInput(const asn1SccFrame& data)
{
    inImage = data;
}

void PoseEstimatorInterface::depthInput(const asn1SccFrame& data)
{
    inDepth = data;
}

void PoseEstimatorInterface::primitivesInput(const asn1SccVectorXdSequence& data)
{
    inPrimitives = data;
}

const asn1SccPosesSequence& PoseEstimatorInterface::posesOutput() const
{
    return outPoses;
}

}
}

/** @} */
