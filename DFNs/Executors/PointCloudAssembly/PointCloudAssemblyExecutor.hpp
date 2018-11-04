/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDASSEMBLY_EXECUTOR_HPP
#define POINTCLOUDASSEMBLY_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include <PointCloudAssembly/PointCloudAssemblyInterface.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Pose.hpp>

namespace CDFF
{
namespace DFN
{
namespace Executors
{

/**
* All the methods in this file execute the DFN for the assembly of point cloud. A DFN instance has to be passed in the constructor of these class.
* The first four methods take the following parameters:
* @param inputFirstCloud: first input point cloud;
* @param inputSecondCloud: second input point cloud;
* @param outputAssembledCloud: output assembled point cloud.
*
* The second four methods take the following parameters:
* @param cloud: point cloud to be merged with stored cloud;
* @param viewCenter: visualization center of the output point cloud;
* @param viewRadius: visualization radius of the output point cloud.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/

void Execute(PointCloudAssemblyInterface* dfn, PointCloudWrapper::PointCloudConstPtr inputFirstCloud, PointCloudWrapper::PointCloudConstPtr inputSecondCloud, 
	PointCloudWrapper::PointCloudConstPtr& outputAssembledCloud);
void Execute(PointCloudAssemblyInterface* dfn, PointCloudWrapper::PointCloudConstPtr inputFirstCloud, PointCloudWrapper::PointCloudConstPtr inputSecondCloud, 
	PointCloudWrapper::PointCloudPtr outputAssembledCloud);
void Execute(PointCloudAssemblyInterface* dfn, const PointCloudWrapper::PointCloud& inputFirstCloud, const PointCloudWrapper::PointCloud& inputSecondCloud, 
	PointCloudWrapper::PointCloudConstPtr& outputAssembledCloud);
void Execute(PointCloudAssemblyInterface* dfn, const PointCloudWrapper::PointCloud& inputFirstCloud, const PointCloudWrapper::PointCloud& inputSecondCloud, 
	PointCloudWrapper::PointCloud& outputAssembledCloud);

void Execute(PointCloudAssemblyInterface* dfn, PointCloudWrapper::PointCloudConstPtr cloud, PoseWrapper::Pose3DConstPtr viewCenter, float viewRadius, 
	PointCloudWrapper::PointCloudConstPtr& outputAssembledCloud);
void Execute(PointCloudAssemblyInterface* dfn, PointCloudWrapper::PointCloudConstPtr cloud, PoseWrapper::Pose3DConstPtr viewCenter, float viewRadius, 
	PointCloudWrapper::PointCloudPtr outputAssembledCloud);
void Execute(PointCloudAssemblyInterface* dfn, const PointCloudWrapper::PointCloud& cloud, const PoseWrapper::Pose3D& viewCenter, float viewRadius, 
	PointCloudWrapper::PointCloudConstPtr& outputAssembledCloud);
void Execute(PointCloudAssemblyInterface* dfn, const PointCloudWrapper::PointCloud& cloud, const PoseWrapper::Pose3D& viewCenter, float viewRadius, 
	PointCloudWrapper::PointCloud& outputAssembledCloud);

}
}
}

#endif // POINTCLOUDASSEMBLY_EXECUTOR_HPP

/** @} */
