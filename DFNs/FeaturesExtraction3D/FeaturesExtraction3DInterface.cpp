/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesExtraction3DInterface.hpp"

namespace CDFF
{
namespace DFN
{

FeaturesExtraction3DInterface::FeaturesExtraction3DInterface()
{
    asn1SccPointcloud_Initialize(&inPointcloud);
    asn1SccVisualPointFeatureVector3D_Initialize(&outFeatures);
}

FeaturesExtraction3DInterface::~FeaturesExtraction3DInterface()
{
}

void FeaturesExtraction3DInterface::pointcloudInput(const asn1SccPointcloud& data)
{
    inPointcloud = data;
}

const asn1SccVisualPointFeatureVector3D& FeaturesExtraction3DInterface::featuresOutput() const
{
    return outFeatures;
}

}
}

/** @} */
