/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESMATCHING2D_FEATURESMATCHING2DINTERFACE_HPP
#define FEATURESMATCHING2D_FEATURESMATCHING2DINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/VisualPointFeatureVector2D.h>
#include <Types/C/CorrespondenceMap2D.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that matches 2D keypoints
     */
    class FeaturesMatching2DInterface : public DFNCommonInterface
    {
        public:

            FeaturesMatching2DInterface();
            virtual ~FeaturesMatching2DInterface();

            /**
             * Send value to input port "sourceFeatures"
             * @param sourceFeatures: keypoints extracted from the source 2D image
             */
            virtual void sourceFeaturesInput(const asn1SccVisualPointFeatureVector2D& data);
            /**
             * Send value to input port "sinkFeatures"
             * @param sinkFeatures: keypoints extracted from the sink 2D image
             */
            virtual void sinkFeaturesInput(const asn1SccVisualPointFeatureVector2D& data);

            /**
             * Query value from output port "matches"
             * @return matches: matches between the two sets of keypoints
             */
            virtual const asn1SccCorrespondenceMap2D& matchesOutput() const;

        protected:

            asn1SccVisualPointFeatureVector2D inSourceFeatures;
            asn1SccVisualPointFeatureVector2D inSinkFeatures;
            asn1SccCorrespondenceMap2D outMatches;
    };
}
}

#endif // FEATURESMATCHING2D_FEATURESMATCHING2DINTERFACE_HPP

/** @} */
