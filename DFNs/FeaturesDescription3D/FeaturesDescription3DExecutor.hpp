/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESDESCRIPTION3D_EXECUTOR_HPP
#define FEATURESDESCRIPTION3D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "FeaturesDescription3DInterface.hpp"
#include <PointCloud.hpp>
#include <VisualPointFeatureVector3D.hpp>

namespace CDFF
{
namespace DFN
{
    class FeaturesDescription3DExecutor
    {
        public:

            FeaturesDescription3DExecutor(FeaturesDescription3DInterface* dfn);
            ~FeaturesDescription3DExecutor();

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputVector, 
			VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputVector);

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputVector, 
			VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr outputVector);

	    void Execute(const PointCloudWrapper::PointCloud& inputCloud, const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputVector, 
			VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputVector);

	    void Execute(const PointCloudWrapper::PointCloud& inputCloud, const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputVector, 
			VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& outputVector);

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputVector, 
			PointCloudWrapper::PointCloudConstPtr normalCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputVector);

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputVector, 
			PointCloudWrapper::PointCloudConstPtr normalCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr outputVector);

	    void Execute(const PointCloudWrapper::PointCloud& inputCloud, const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputVector, 
			const PointCloudWrapper::PointCloud& normalCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputVector);

	    void Execute(const PointCloudWrapper::PointCloud& inputCloud, const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputVector, 
			const PointCloudWrapper::PointCloud& normalCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& outputVector);

        private:

            FeaturesDescription3DInterface* dfn;
    };
}
}

#endif // FEATURESDESCRIPTION3D_EXECUTOR_HPP

/** @} */
