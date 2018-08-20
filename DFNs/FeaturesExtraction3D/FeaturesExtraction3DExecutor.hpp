/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESEXTRACTION3D_EXECUTOR_HPP
#define FEATURESEXTRACTION3D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "FeaturesExtraction3DInterface.hpp"
#include <PointCloud.hpp>
#include <VisualPointFeatureVector3D.hpp>

namespace CDFF
{
namespace DFN
{
    class FeaturesExtraction3DExecutor
    {
        public:

            FeaturesExtraction3DExecutor(FeaturesExtraction3DInterface* dfn);
            ~FeaturesExtraction3DExecutor();

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputVector);
	    void Execute(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr outputVector);
	    void Execute(const PointCloudWrapper::PointCloud& inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputVector);
	    void Execute(const PointCloudWrapper::PointCloud& inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& outputVector);

        private:

            FeaturesExtraction3DInterface* dfn;
    };
}
}

#endif // FEATURESEXTRACTION3D_EXECUTOR_HPP

/** @} */
