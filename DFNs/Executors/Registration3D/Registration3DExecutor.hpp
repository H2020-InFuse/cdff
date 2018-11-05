/**
 * @addtogroup DFNs
 * @{
 */

#ifndef REGISTRATION3D_EXECUTOR_HPP
#define REGISTRATION3D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include <Registration3D/Registration3DInterface.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Pose.hpp>

namespace CDFF
{
namespace DFN
{
namespace Executors
{
/**
* All the methods in this file execute the DFN for the computation of the position of the sink point cloud in the reference system of the source point cloud. 
* A DFN instance has to be passed in the constructor of these class. 
* The first four methods take the following parameters:
* @param inputSourceCloud: input source cloud;
* @param inputSinkCloud: input sink cloud;
* @param outputTransform: pose of the source cloud in the reference of the sink cloud;
* @param success: boolean telling whether the computation was successfull.
*
* The second four methods take the following parameters:
* @param inputSourceCloud: input source cloud;
* @param inputSinkCloud: input sink cloud;
* @param poseGuess: initial estimation of the second cloud in the reference of the first cloud;
* @param outputTransform: pose of the source cloud in the reference of the sink cloud;
* @param success: boolean telling whether the computation was successfull.
*
* The main difference within each group of four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/
void Execute(Registration3DInterface* dfn, PointCloudWrapper::PointCloudConstPtr inputSourceCloud, PointCloudWrapper::PointCloudConstPtr inputSinkCloud, 
	PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

void Execute(Registration3DInterface* dfn, PointCloudWrapper::PointCloudConstPtr inputSourceCloud, PointCloudWrapper::PointCloudConstPtr inputSinkCloud, 
	PoseWrapper::Pose3DPtr outputTransform, bool& success);

void Execute(Registration3DInterface* dfn, const PointCloudWrapper::PointCloud& inputSourceCloud, const PointCloudWrapper::PointCloud& inputSinkCloud, 
	PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

void Execute(Registration3DInterface* dfn, const PointCloudWrapper::PointCloud& inputSourceCloud, const PointCloudWrapper::PointCloud& inputSinkCloud, 
	PoseWrapper::Pose3D& outputTransform, bool& success);

void Execute(Registration3DInterface* dfn, PointCloudWrapper::PointCloudConstPtr inputSourceCloud, PointCloudWrapper::PointCloudConstPtr inputSinkCloud, 
	PoseWrapper::Pose3DConstPtr poseGuess, PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

void Execute(Registration3DInterface* dfn, PointCloudWrapper::PointCloudConstPtr inputSourceCloud, PointCloudWrapper::PointCloudConstPtr inputSinkCloud, 
	PoseWrapper::Pose3DConstPtr poseGuess, PoseWrapper::Pose3DPtr outputTransform, bool& success);

void Execute(Registration3DInterface* dfn, const PointCloudWrapper::PointCloud& inputSourceCloud, const PointCloudWrapper::PointCloud& inputSinkCloud, 
	const PoseWrapper::Pose3D& poseGuess, PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

void Execute(Registration3DInterface* dfn, const PointCloudWrapper::PointCloud& inputSourceCloud, const PointCloudWrapper::PointCloud& inputSinkCloud, 
	const PoseWrapper::Pose3D& poseGuess, PoseWrapper::Pose3D& outputTransform, bool& success);

}
}
}

#endif // REGISTRATION3D_EXECUTOR_HPP

/** @} */
