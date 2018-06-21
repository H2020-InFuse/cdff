/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesMatching2DInterface.hpp"

namespace dfn_ci
{

FeaturesMatching2DInterface::FeaturesMatching2DInterface()
{
}

FeaturesMatching2DInterface::~FeaturesMatching2DInterface()
{
}

void FeaturesMatching2DInterface::sourceFeaturesInput(const asn1SccVisualPointFeatureVector2D& data)
{
    inSourceFeatures = data;
}

void FeaturesMatching2DInterface::sinkFeaturesInput(const asn1SccVisualPointFeatureVector2D& data)
{
    inSinkFeatures = data;
}

const asn1SccCorrespondenceMap2D& FeaturesMatching2DInterface::matchesOutput() const
{
    return outMatches;
}

}

/** @} */
