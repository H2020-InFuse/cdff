/**
 * @addtogroup DFNs
 * @{
 */

#include "EdgeDetectionInterface.hpp"

namespace CDFF
{
namespace DFN
{

EdgeDetectionInterface::EdgeDetectionInterface()
{
}

EdgeDetectionInterface::~EdgeDetectionInterface()
{
}

void EdgeDetectionInterface::imageInput(const asn1SccFrame& data)
{
    inImage = data;
}

const asn1SccFrame& EdgeDetectionInterface::edgeMapOutput() const
{
    return outEdgeMap;
}

const asn1SccFrame& EdgeDetectionInterface::sobelGradientXOutput() const
{
    return outSobelGradientX;
}

const asn1SccFrame& EdgeDetectionInterface::sobelGradientYOutput() const
{
    return outSobelGradientY;
}

}
}

/** @} */
