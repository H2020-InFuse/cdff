/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesDescription3DInterface.hpp"

namespace CDFF
{
namespace DFN
{

FeaturesDescription3DInterface::FeaturesDescription3DInterface()
{
    asn1SccPointcloud_Initialize(&inPointcloud) ;
    asn1SccVisualPointFeatureVector3D_Initialize(&inFeatures) ;
    asn1SccPointcloud_Initialize(&inNormals) ;
    asn1SccVisualPointFeatureVector3D_Initialize(&outFeatures) ;
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
}

/** @} */
