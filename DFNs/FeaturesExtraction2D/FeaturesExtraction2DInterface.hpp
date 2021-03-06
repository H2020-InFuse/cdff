/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESEXTRACTION2D_FEATURESEXTRACTION2DINTERFACE_HPP
#define FEATURESEXTRACTION2D_FEATURESEXTRACTION2DINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/VisualPointFeatureVector2D.h>
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that extracts 2D keypoints from a 2D image
     */
    class FeaturesExtraction2DInterface : public DFNCommonInterface
    {
        public:

            FeaturesExtraction2DInterface();
            virtual ~FeaturesExtraction2DInterface();

            /**
             * Send value to input port "frame"
             * @param frame: 2D image captured by a camera
             */
            virtual void frameInput(const asn1SccFrame& data);

            /**
             * Query value from output port "features"
             * @return features: keypoints extracted from the image, may include descriptors
             */
            virtual const asn1SccVisualPointFeatureVector2D& featuresOutput() const;

        protected:

            asn1SccFrame inFrame;
            asn1SccVisualPointFeatureVector2D outFeatures;
    };
}
}

#endif // FEATURESEXTRACTION2D_FEATURESEXTRACTION2DINTERFACE_HPP

/** @} */
