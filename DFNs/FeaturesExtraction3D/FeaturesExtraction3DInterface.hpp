/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESEXTRACTION3D_FEATURESEXTRACTION3DINTERFACE_HPP
#define FEATURESEXTRACTION3D_FEATURESEXTRACTION3DINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Pointcloud.h>
#include <Types/C/VisualPointFeatureVector3D.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that extracts 3D keypoints from a 3D pointcloud
     */
    class FeaturesExtraction3DInterface : public DFNCommonInterface
    {
        public:

            FeaturesExtraction3DInterface();
            virtual ~FeaturesExtraction3DInterface();

            /**
             * Send value to input port "pointcloud"
             * @param pointcloud: 3D pointcloud captured by a 3D sensor
             *        or reconstructed from other perceptions
             */
            virtual void pointcloudInput(const asn1SccPointcloud& data);

            /**
             * Query value from output port "features"
             * @return features: keypoints extracted from the pointcloud,
             *         may include descriptors
             */
            virtual const asn1SccVisualPointFeatureVector3D& featuresOutput() const;

        protected:

            asn1SccPointcloud inPointcloud;
            asn1SccVisualPointFeatureVector3D outFeatures;
    };
}
}

#endif // FEATURESEXTRACTION3D_FEATURESEXTRACTION3DINTERFACE_HPP

/** @} */
