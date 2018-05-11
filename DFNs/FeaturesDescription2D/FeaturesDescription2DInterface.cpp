/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesDescription2DInterface.hpp"

namespace dfn_ci
{

FeaturesDescription2DInterface::FeaturesDescription2DInterface()
{
}

FeaturesDescription2DInterface::~FeaturesDescription2DInterface()
{
}

void FeaturesDescription2DInterface::frameInput(const CTypes::Frame& data)
{
    inFrame = data;
}

void FeaturesDescription2DInterface::featuresInput(const CTypes::VisualPointFeatureVector2D& data)
{
    inFeatures = data;
}

const CTypes::VisualPointFeatureVector2D& FeaturesDescription2DInterface::featuresOutput() const
{
    return outFeatures;
}

}

/** @} */
