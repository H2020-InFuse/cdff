/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesExtraction2DInterface.hpp"

namespace dfn_ci
{

FeaturesExtraction2DInterface::FeaturesExtraction2DInterface()
{
}

FeaturesExtraction2DInterface::~FeaturesExtraction2DInterface()
{
}

void FeaturesExtraction2DInterface::frameInput(const asn1SccFrame& data)
{
    inFrame = data;
}

const asn1SccVisualPointFeatureVector2D& FeaturesExtraction2DInterface::featuresOutput() const
{
    return outFeatures;
}

}

/** @} */
