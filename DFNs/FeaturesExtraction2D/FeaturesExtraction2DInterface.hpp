/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESEXTRACTION2D_INTERFACE_HPP
#define FEATURESEXTRACTION2D_INTERFACE_HPP

#include "DFNCommonInterface.hpp"

namespace CTypes {
#include <VisualPointFeatureVector2D.h>
#include <Frame.h>
}

namespace dfn_ci
{
    /**
     * Common interface for all DFNs that extract 2D keypoints from a 2D image
     */
    class FeaturesExtraction2DInterface : public DFNCommonInterface
    {
        public:

            FeaturesExtraction2DInterface();
            virtual ~FeaturesExtraction2DInterface();

            /**
             * Send value to input port "frame"
             * @param frame, 2D image captured by a camera
             */
            virtual void frameInput(const CTypes::Frame& data);

            /**
             * Query value from output port "features"
             * @return features, keypoints extracted from the image, may include descriptors
             */
            virtual const CTypes::VisualPointFeatureVector2D& featuresOutput() const;

        protected:

            CTypes::Frame inFrame;
            CTypes::VisualPointFeatureVector2D outFeatures;
    };
}

#endif // FEATURESEXTRACTION2D_INTERFACE_HPP

/** @} */
