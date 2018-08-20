/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESMATCHING2D_EXECUTOR_HPP
#define FEATURESMATCHING2D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "FeaturesMatching2DInterface.hpp"
#include <VisualPointFeatureVector2D.hpp>
#include <CorrespondenceMap2D.hpp>

namespace CDFF
{
namespace DFN
{
    class FeaturesMatching2DExecutor
    {
        public:

            FeaturesMatching2DExecutor(FeaturesMatching2DInterface* dfn);
            ~FeaturesMatching2DExecutor();

	    void Execute(VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputSourceVector, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputSinkVector, 
			CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr& outputMatches);

	    void Execute(VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputSourceVector, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputSinkVector, 
			CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr outputMatches);

	    void Execute(const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputSourceVector, const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputSinkVector, 
			CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr& outputMatches);

	    void Execute(const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputSourceVector, const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputSinkVector, 
			CorrespondenceMap2DWrapper::CorrespondenceMap2D& outputMatches);

        private:

            FeaturesMatching2DInterface* dfn;
    };
}
}

#endif // FEATURESMATCHING2D_EXECUTOR_HPP

/** @} */
