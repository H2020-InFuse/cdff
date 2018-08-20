/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDRECONSTRUCTION2DTO3D_EXECUTOR_HPP
#define POINTCLOUDRECONSTRUCTION2DTO3D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "PointCloudReconstruction2DTo3DInterface.hpp"
#include <Pose.hpp>
#include <CorrespondenceMap2D.hpp>
#include <PointCloud.hpp>

namespace CDFF
{
namespace DFN
{
    class PointCloudReconstruction2DTo3DExecutor
    {
        public:

            PointCloudReconstruction2DTo3DExecutor(PointCloudReconstruction2DTo3DInterface* dfn);
            ~PointCloudReconstruction2DTo3DExecutor();

	    void Execute(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputMatches, PoseWrapper::Pose3DConstPtr inputPose, PointCloudWrapper::PointCloudConstPtr& outputCloud);

	    void Execute(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputMatches, PoseWrapper::Pose3DConstPtr inputPose, PointCloudWrapper::PointCloudPtr outputCloud);

	    void Execute(const CorrespondenceMap2DWrapper::CorrespondenceMap2D& inputMatches, const PoseWrapper::Pose3D& inputPose, PointCloudWrapper::PointCloudConstPtr& outputCloud);

	    void Execute(const CorrespondenceMap2DWrapper::CorrespondenceMap2D& inputMatches, const PoseWrapper::Pose3D& inputPose, PointCloudWrapper::PointCloud& outputCloud);

        private:

            PointCloudReconstruction2DTo3DInterface* dfn;
    };
}
}
#endif // POINTCLOUDRECONSTRUCTION2DTO3D_EXECUTOR_HPP

/** @} */
