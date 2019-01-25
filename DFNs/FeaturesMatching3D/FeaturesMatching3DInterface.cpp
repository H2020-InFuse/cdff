/**
 * @addtogroup DFNs
 * @{
 */

#include "FeaturesMatching3DInterface.hpp"

namespace CDFF
{
namespace DFN
{

FeaturesMatching3DInterface::FeaturesMatching3DInterface()
{
    asn1SccVisualPointFeatureVector3D_Initialize(&inSourceFeatures);
    asn1SccVisualPointFeatureVector3D_Initialize(& inSinkFeatures);
    asn1SccPose_Initialize(& outTransform);
}

FeaturesMatching3DInterface::~FeaturesMatching3DInterface()
{
}

void FeaturesMatching3DInterface::sourceFeaturesInput(const asn1SccVisualPointFeatureVector3D& data)
{
    inSourceFeatures = data;
}

void FeaturesMatching3DInterface::sinkFeaturesInput(const asn1SccVisualPointFeatureVector3D& data)
{
    inSinkFeatures = data;
}

const asn1SccPose& FeaturesMatching3DInterface::transformOutput() const
{
    return outTransform;
}

bool FeaturesMatching3DInterface::successOutput() const
{
    return outSuccess;
}

}
}

/** @} */
