/**
 * @addtogroup DFNs
 * @{
 */

#include "Registration3DInterface.hpp"

namespace dfn_ci
{

Registration3DInterface::Registration3DInterface()
{
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

const asn1SccPose& Registration3DInterface::transformOutput() const
{
    return outTransform;
}

bool Registration3DInterface::successOutput() const
{
    return outSuccess;
}

}

/** @} */
