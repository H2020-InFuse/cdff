/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesExtraction2DInterface.hpp"

namespace CDFF
{
namespace DFN
{

FeaturesExtraction2DInterface::FeaturesExtraction2DInterface()
{
    asn1SccFrame_Initialize(&inFrame) ;
    asn1SccVisualPointFeatureVector2D_Initialize(&outFeatures) ;
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
}

/** @} */
