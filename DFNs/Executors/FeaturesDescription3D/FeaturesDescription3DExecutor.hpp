/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESDESCRIPTION3D_EXECUTOR_HPP
#define FEATURESDESCRIPTION3D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "FeaturesDescription3D/FeaturesDescription3DInterface.hpp"
#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>

namespace CDFF
{
namespace DFN
{
namespace Executors
{
/**
* All the methods in this file execute the DFN for the computation of the features descriptors of 3d point clouds.  A DFN instance has to be passed in the constructor of these class. 
* The first four methods take the following parameters:
* @param inputCloud: input point cloud;
* @param inputVector: input vector of keypoints;
* @param outputVector: output vector of keypoints with associated descriptors;
*
* The second four methods take the following parameters:
* @param inputCloud: input point cloud;
* @param inputVector: input vector of keypoints;
* @param normalCloud: input point cloud of normals (it is a point cloud where each i-th point is actually the normal of inputCloud at position i);
* @param outputVector: output vector of keypoints with associated descriptors;
*
* The main difference within each group of four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/

void Execute(FeaturesDescription3DInterface* dfn, PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputVector, 
	VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputVector);

void Execute(FeaturesDescription3DInterface* dfn, PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputVector, 
	VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr outputVector);

void Execute(FeaturesDescription3DInterface* dfn, const PointCloudWrapper::PointCloud& inputCloud, const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputVector, 
	VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputVector);

void Execute(FeaturesDescription3DInterface* dfn, const PointCloudWrapper::PointCloud& inputCloud, const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputVector, 
	VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& outputVector);

void Execute(FeaturesDescription3DInterface* dfn, PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputVector, 
	PointCloudWrapper::PointCloudConstPtr normalCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputVector);

void Execute(FeaturesDescription3DInterface* dfn, PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputVector, 
	PointCloudWrapper::PointCloudConstPtr normalCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr outputVector);

void Execute(FeaturesDescription3DInterface* dfn, const PointCloudWrapper::PointCloud& inputCloud, const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputVector, 
	const PointCloudWrapper::PointCloud& normalCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputVector);

void Execute(FeaturesDescription3DInterface* dfn, const PointCloudWrapper::PointCloud& inputCloud, const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputVector, 
	const PointCloudWrapper::PointCloud& normalCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& outputVector);

}
}
}

#endif // FEATURESDESCRIPTION3D_EXECUTOR_HPP

/** @} */
