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

void FeaturesExtraction2DInterface::frameInput(const CTypes::Frame& data)
{
    inFrame = data;
}

const CTypes::VisualPointFeatureVector2D& FeaturesExtraction2DInterface::featuresOutput() const
{
    return outFeatures;
}

}

/** @} */
