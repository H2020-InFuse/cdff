/**
 * @addtogroup DFNs
 * @{
 */

#include "Registration3DInterface.hpp"

namespace CDFF
{
namespace DFN
{

Registration3DInterface::Registration3DInterface()
{
    asn1SccPointcloud_Initialize(& inSourceCloud);
    asn1SccPointcloud_Initialize(& inSinkCloud);
    asn1SccPose_Initialize(& inTransformGuess);
    asn1SccPose_Initialize(& outTransform);
}

Registration3DInterface::~Registration3DInterface()
{
}

void Registration3DInterface::sourceCloudInput(const asn1SccPointcloud& data)
{
    inSourceCloud = data;
}

void Registration3DInterface::sinkCloudInput(const asn1SccPointcloud& data)
{
    inSinkCloud = data;
}

void Registration3DInterface::transformGuessInput(const asn1SccPose& data)
{
    inTransformGuess = data;
}

void Registration3DInterface::useGuessInput(const bool& data)
{
    inUseGuess = data;
}

const asn1SccPose& Registration3DInterface::transformOutput() const
{
    return outTransform;
}

bool Registration3DInterface::successOutput() const
{
    return outSuccess;
}

}
}

/** @} */
