/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesDescription2DInterface.hpp"

namespace CDFF
{
namespace DFN
{

FeaturesDescription2DInterface::FeaturesDescription2DInterface()
{
    asn1SccFrame_Initialize(&inFrame) ;
    asn1SccVisualPointFeatureVector2D_Initialize(&inFeatures) ;
    asn1SccVisualPointFeatureVector2D_Initialize(&outFeatures) ;
}

FeaturesDescription2DInterface::~FeaturesDescription2DInterface()
{
}

void FeaturesDescription2DInterface::frameInput(const asn1SccFrame& data)
{
    inFrame = data;
}

void FeaturesDescription2DInterface::featuresInput(const asn1SccVisualPointFeatureVector2D& data)
{
    inFeatures = data;
}

const asn1SccVisualPointFeatureVector2D& FeaturesDescription2DInterface::featuresOutput() const
{
    return outFeatures;
}

}
}

/** @} */
