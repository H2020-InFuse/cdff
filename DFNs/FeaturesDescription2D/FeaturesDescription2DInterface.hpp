/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESDESCRIPTION2D_INTERFACE_HPP
#define FEATURESDESCRIPTION2D_INTERFACE_HPP

#include "DFNCommonInterface.hpp"

namespace CTypes {
#include <VisualPointFeatureVector2D.h>
#include <Frame.h>
}

namespace dfn_ci
{
    /**
     * Common interface for all DFNs that compute descriptors for 2D keypoints
     */
    class FeaturesDescription2DInterface : public DFNCommonInterface
    {
        public:

            FeaturesDescription2DInterface();
            virtual ~FeaturesDescription2DInterface();

            /**
             * Send value to input port "frame"
             * @param frame, 2D image captured by a camera
             */
            virtual void frameInput(const CTypes::Frame& data);
            /**
             * Send value to input port "features"
             * @param features, keypoints extracted from the image, without any descriptors
             */
            virtual void featuresInput(const CTypes::VisualPointFeatureVector2D& data);

            /**
             * Query value from output port "features"
             * @return features, same keypoints, with added descriptors
             */
            virtual const CTypes::VisualPointFeatureVector2D& featuresOutput() const;

        protected:

            CTypes::Frame inFrame;
            CTypes::VisualPointFeatureVector2D inFeatures;
            CTypes::VisualPointFeatureVector2D outFeatures;
    };
}

#endif // FEATURESDESCRIPTION2D_INTERFACE_HPP

/** @} */
