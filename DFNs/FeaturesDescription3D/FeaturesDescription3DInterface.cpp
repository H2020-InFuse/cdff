/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesDescription3DInterface.hpp"

namespace dfn_ci
{

FeaturesDescription3DInterface::FeaturesDescription3DInterface()
{
}

FeaturesDescription3DInterface::~FeaturesDescription3DInterface()
{
}

void FeaturesDescription3DInterface::pointcloudInput(const asn1SccPointcloud& data)
{
    inPointcloud = data;
}

void FeaturesDescription3DInterface::featuresInput(const asn1SccVisualPointFeatureVector3D& data)
{
    inFeatures = data;
}

void FeaturesDescription3DInterface::normalsInput(const asn1SccPointcloud& data)
{
    inNormals = data;
}

const asn1SccVisualPointFeatureVector3D& FeaturesDescription3DInterface::featuresOutput() const
{
    return outFeatures;
}

}

/** @} */
