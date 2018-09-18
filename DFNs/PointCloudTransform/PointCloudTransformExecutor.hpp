/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDTRANSFORM_EXECUTOR_HPP
#define POINTCLOUDTRANSFORM_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "PointCloudTransformInterface.hpp"
#include <PointCloud.hpp>
#include <Pose.hpp>

namespace CDFF
{
namespace DFN
{
/**
* All the methods in this class execute the DFN for the assembly of point cloud. A DFN instance has to be passed in the constructor of these class.
* The first four methods take the following parameters:
* @param inputCloud: input point cloud;
* @param inputPose: pose of point cloud system in system external system E;
* @param outputCloud: input point cloud as seen in E.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/
    class PointCloudTransformExecutor
    {
        public:

            PointCloudTransformExecutor(PointCloudTransformInterface* dfn);
            ~PointCloudTransformExecutor();

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputCloud, PoseWrapper::Pose3DConstPtr inputPose, PointCloudWrapper::PointCloudConstPtr& outputCloud);
	    void Execute(PointCloudWrapper::PointCloudConstPtr inputCloud, PoseWrapper::Pose3DConstPtr inputPose, PointCloudWrapper::PointCloudPtr outputCloud);
	    void Execute(const PointCloudWrapper::PointCloud& inputCloud, const PoseWrapper::Pose3D& inputPose, PointCloudWrapper::PointCloudConstPtr& outputCloud);
	    void Execute(const PointCloudWrapper::PointCloud& inputCloud, const PoseWrapper::Pose3D& inputPose, PointCloudWrapper::PointCloud& outputCloud);

        private:

            PointCloudTransformInterface* dfn;
    };
}
}

#endif // POINTCLOUDTRANSFORM_EXECUTOR_HPP

/** @} */
