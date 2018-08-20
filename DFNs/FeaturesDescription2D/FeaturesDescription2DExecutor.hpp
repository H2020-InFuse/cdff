/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESDESCRIPTION2D_EXECUTOR_HPP
#define FEATURESDESCRIPTION2D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "FeaturesDescription2DInterface.hpp"
#include <VisualPointFeatureVector2D.h>
#include <Frame.h>
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>

namespace CDFF
{
namespace DFN
{
    class FeaturesDescription2DExecutor
    {
        public:

            FeaturesDescription2DExecutor(FeaturesDescription2DInterface* dfn);
            ~FeaturesDescription2DExecutor();

	    void Execute(FrameWrapper::FrameConstPtr inputFrame, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputVector, 
			VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr& outputVector);

	    void Execute(FrameWrapper::FrameConstPtr inputFrame, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputVector, 
			VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr outputVector);

	    void Execute(const FrameWrapper::Frame& inputFrame, const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputVector, 
			VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr& outputVector);

	    void Execute(const FrameWrapper::Frame& inputFrame, const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputVector, 
			VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& outputVector);

        private:

            FeaturesDescription2DInterface* dfn;
    };
}
}

#endif // FEATURESDESCRIPTION2D_EXECUTOR_HPP

/** @} */
