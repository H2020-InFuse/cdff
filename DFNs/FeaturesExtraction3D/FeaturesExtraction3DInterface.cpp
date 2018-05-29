/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesExtraction3DInterface.hpp"

namespace dfn_ci
{

FeaturesExtraction3DInterface::FeaturesExtraction3DInterface()
{
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

/** @} */
