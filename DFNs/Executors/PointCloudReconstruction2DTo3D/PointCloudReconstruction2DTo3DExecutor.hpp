/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDRECONSTRUCTION2DTO3D_EXECUTOR_HPP
#define POINTCLOUDRECONSTRUCTION2DTO3D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>
#include <Pose.hpp>
#include <CorrespondenceMap2D.hpp>
#include <PointCloud.hpp>

namespace CDFF
{
namespace DFN
{
namespace Executors
{

/**
* All the methods in this file execute the DFN for the computation of a point cloud from 2d feature matches and camera pose. 
* A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputMatches: input matches between 2d features of images taken by two cameras;
* @param inputPose: pose of the second camera in the reference system of the first camera;
* @param outputCloud: output point cloud.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/

void Execute(PointCloudReconstruction2DTo3DInterface* dfn, CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputMatches, PoseWrapper::Pose3DConstPtr inputPose, 
	PointCloudWrapper::PointCloudConstPtr& outputCloud);
void Execute(PointCloudReconstruction2DTo3DInterface* dfn, CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputMatches, PoseWrapper::Pose3DConstPtr inputPose, 
	PointCloudWrapper::PointCloudPtr outputCloud);
void Execute(PointCloudReconstruction2DTo3DInterface* dfn, const CorrespondenceMap2DWrapper::CorrespondenceMap2D& inputMatches, const PoseWrapper::Pose3D& inputPose, 
	PointCloudWrapper::PointCloudConstPtr& outputCloud);
void Execute(PointCloudReconstruction2DTo3DInterface* dfn, const CorrespondenceMap2DWrapper::CorrespondenceMap2D& inputMatches, const PoseWrapper::Pose3D& inputPose, 
	PointCloudWrapper::PointCloud& outputCloud);

}
}
}
#endif // POINTCLOUDRECONSTRUCTION2DTO3D_EXECUTOR_HPP

/** @} */
