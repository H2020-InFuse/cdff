/**
 * @author Irene Sanz
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "ThresholdForce.hpp"
#include <Errors/Assert.hpp>

namespace CDFF
{
namespace DFN
{
namespace ForceMeshGenerator
{

//=====================================================================================================================
ThresholdForce::ThresholdForce()
{
    parametersHelper.AddParameter<double>("GeneralParameters", "Threshold", parameters.threshold, DEFAULT_PARAMETERS.threshold);
	configurationFilePath = "";
}

//=====================================================================================================================
const ThresholdForce::ThresholdForceOptionsSet ThresholdForce::DEFAULT_PARAMETERS =
{
    /*.threshold =*/ 0.5
};

//=====================================================================================================================
void ThresholdForce::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

//=====================================================================================================================
void ThresholdForce::process()
{
	// Process data
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr outPCLPointCloud = CreatePointCloud();

	// Write data to output port
	PointCloudWrapper::PointCloudConstPtr tmp = pclPointCloudToPointCloud.Convert(outPCLPointCloud);
    PointCloudWrapper::Copy(*tmp, outPointCloud);
}

//=====================================================================================================================
pcl::PointCloud<pcl::PointXYZ>::Ptr ThresholdForce::CreatePointCloud()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZ>());

	Eigen::Vector3d position(inRoverPose.pos.arr[0], inRoverPose.pos.arr[1], inRoverPose.pos.arr[2]);
	Eigen::Quaterniond quaternion(inRoverPose.orient.arr[0], inRoverPose.orient.arr[1], inRoverPose.orient.arr[2], inRoverPose.orient.arr[3]);
	quaternion.normalize();
	Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

	unsigned int size = inPositions.nCount;
	for( unsigned int index = 0; index < size; index ++ )
	{
		if( inForces.arr[index] > parameters.threshold )
		{
			Eigen::Vector3d point_eigen(inPositions.arr[index].arr[0], inPositions.arr[index].arr[1], inPositions.arr[index].arr[2]);
			point_eigen += position;
			point_eigen = rotation * point_eigen;

			pcl::PointXYZ point (point_eigen[0], point_eigen[1], point_eigen[2]) ;
			new_cloud->points.push_back(point);
		}
	}

	return new_cloud;
}

//=====================================================================================================================
void ThresholdForce::ValidateParameters()
{
}

//=====================================================================================================================
void ThresholdForce::ValidateInputs()
{
}


}
}
}

/** @} */
