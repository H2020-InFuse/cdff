/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESEXTRACTION2D_EXECUTOR_HPP
#define FEATURESEXTRACTION2D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "FeaturesExtraction2DInterface.hpp"
#include <VisualPointFeatureVector2D.h>
#include <Frame.h>
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>

namespace dfn_ci
{
    class FeaturesExtraction2DExecutor
    {
        public:

            FeaturesExtraction2DExecutor(FeaturesExtraction2DInterface* dfn);
            ~FeaturesExtraction2DExecutor();

	    void Execute(FrameWrapper::FrameConstPtr inputFrame, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr& outputVector);
	    void Execute(FrameWrapper::FrameConstPtr inputFrame, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr outputVector);
	    void Execute(const FrameWrapper::Frame& inputFrame, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr& outputVector);
	    void Execute(const FrameWrapper::Frame& inputFrame, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& outputVector);

        private:

            FeaturesExtraction2DInterface* dfn;
    };
}

#endif // FEATURESEXTRACTION2D_INTERFACE_HPP

/** @} */
