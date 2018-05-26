/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESDESCRIPTION2D_INTERFACE_HPP
#define FEATURESDESCRIPTION2D_INTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <VisualPointFeatureVector2D.h>
#include <Frame.h>

namespace dfn_ci
{
    /**
     * DFN that computes descriptors for 2D keypoints
     */
    class FeaturesDescription2DInterface : public DFNCommonInterface
    {
        public:

            FeaturesDescription2DInterface();
            virtual ~FeaturesDescription2DInterface();

            /**
             * Send value to input port "frame"
             * @param frame: 2D image captured by a camera
             */
            virtual void frameInput(const asn1SccFrame& data);
            /**
             * Send value to input port "features"
             * @param features: keypoints extracted from the image, without any descriptors
             */
            virtual void featuresInput(const asn1SccVisualPointFeatureVector2D& data);

            /**
             * Query value from output port "features"
             * @return features: same keypoints, with added descriptors
             */
            virtual const asn1SccVisualPointFeatureVector2D& featuresOutput() const;

        protected:

            asn1SccFrame inFrame;
            asn1SccVisualPointFeatureVector2D inFeatures;
            asn1SccVisualPointFeatureVector2D outFeatures;
    };
}

#endif // FEATURESDESCRIPTION2D_INTERFACE_HPP

/** @} */
