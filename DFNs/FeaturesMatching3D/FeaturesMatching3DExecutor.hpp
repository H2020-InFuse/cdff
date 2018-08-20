/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESMATCHING3D_EXECUTOR_HPP
#define FEATURESMATCHING3D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "FeaturesMatching3DInterface.hpp"
#include <VisualPointFeatureVector3D.hpp>
#include <Pose.hpp>

namespace CDFF
{
namespace DFN
{
    class FeaturesMatching3DExecutor
    {
        public:

            FeaturesMatching3DExecutor(FeaturesMatching3DInterface* dfn);
            ~FeaturesMatching3DExecutor();

	    void Execute(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputSourceVector, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputSinkVector, 
			PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

	    void Execute(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputSourceVector, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputSinkVector, 
			PoseWrapper::Pose3DPtr outputTransform, bool& success);

	    void Execute(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputSourceVector, const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputSinkVector, 
			PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

	    void Execute(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputSourceVector, const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputSinkVector, 
			PoseWrapper::Pose3D& outputTransform, bool& success);

        private:

            FeaturesMatching3DInterface* dfn;
    };
}
}

#endif // FEATURESMATCHING3D_EXECUTOR_HPP

/** @} */
