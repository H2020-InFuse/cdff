/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesMatching2DInterface.hpp"

namespace CDFF
{
namespace DFN
{

FeaturesMatching2DInterface::FeaturesMatching2DInterface()
{
    asn1SccVisualPointFeatureVector2D_Initialize(&inSourceFeatures) ;
    asn1SccVisualPointFeatureVector2D_Initialize(&inSinkFeatures) ;
    asn1SccCorrespondenceMap2D_Initialize(&outMatches) ;
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
}

/** @} */
