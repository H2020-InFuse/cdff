/**
 * @addtogroup DFNs
 * @{
 */

#ifndef REGISTRATION3D_EXECUTOR_HPP
#define REGISTRATION3D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "Registration3DInterface.hpp"
#include <PointCloud.hpp>
#include <Pose.hpp>

namespace dfn_ci
{

    class Registration3DExecutor
    {
        public:

            Registration3DExecutor(Registration3DInterface* dfn);
            ~Registration3DExecutor();

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputSourceCloud, PointCloudWrapper::PointCloudConstPtr inputSinkCloud, 
			PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputSourceCloud, PointCloudWrapper::PointCloudConstPtr inputSinkCloud, 
			PoseWrapper::Pose3DPtr outputTransform, bool& success);

	    void Execute(const PointCloudWrapper::PointCloud& inputSourceCloud, const PointCloudWrapper::PointCloud& inputSinkCloud, 
			PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

	    void Execute(const PointCloudWrapper::PointCloud& inputSourceCloud, const PointCloudWrapper::PointCloud& inputSinkCloud, 
			PoseWrapper::Pose3D& outputTransform, bool& success);

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputSourceCloud, PointCloudWrapper::PointCloudConstPtr inputSinkCloud, PoseWrapper::Pose3DConstPtr poseGuess,
			PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputSourceCloud, PointCloudWrapper::PointCloudConstPtr inputSinkCloud, PoseWrapper::Pose3DConstPtr poseGuess,
			PoseWrapper::Pose3DPtr outputTransform, bool& success);

	    void Execute(const PointCloudWrapper::PointCloud& inputSourceCloud, const PointCloudWrapper::PointCloud& inputSinkCloud, const PoseWrapper::Pose3D& poseGuess,
			PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

	    void Execute(const PointCloudWrapper::PointCloud& inputSourceCloud, const PointCloudWrapper::PointCloud& inputSinkCloud, const PoseWrapper::Pose3D& poseGuess,
			PoseWrapper::Pose3D& outputTransform, bool& success);

        private:

            Registration3DInterface* dfn;
    };
}

#endif // REGISTRATION3D_EXECUTOR_HPP

/** @} */
