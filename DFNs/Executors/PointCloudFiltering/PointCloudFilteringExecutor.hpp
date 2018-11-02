/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDFILTERING_EXECUTOR_HPP
#define POINTCLOUDFILTERING_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include <PointCloudFiltering/PointCloudFilteringInterface.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>

namespace CDFF
{
namespace DFN
{
namespace Executors
{

/**
* All the methods in this file execute the DFN for the filtering of a point cloud. A DFN instance has to be passed in the constructor of these class.
* The first four methods take the following parameters:
* @param inputCloud: input point cloud;
* @param outputCloud: input point cloud after filter is applied.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/

void Execute(PointCloudFilteringInterface* dfn, PointCloudWrapper::PointCloudConstPtr inputCloud, PointCloudWrapper::PointCloudConstPtr& outputCloud);
void Execute(PointCloudFilteringInterface* dfn, PointCloudWrapper::PointCloudConstPtr inputCloud, PointCloudWrapper::PointCloudPtr outputCloud);
void Execute(PointCloudFilteringInterface* dfn, const PointCloudWrapper::PointCloud& inputCloud, PointCloudWrapper::PointCloudConstPtr& outputCloud);
void Execute(PointCloudFilteringInterface* dfn, const PointCloudWrapper::PointCloud& inputCloud, PointCloudWrapper::PointCloud& outputCloud);

}
}
}

#endif // POINTCLOUDFILTERING_EXECUTOR_HPP

/** @} */
