/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "Icp3D.hpp"

#include <PointCloudToPclPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>
#include <pcl/registration/icp.h>

#include <stdlib.h>
#include <fstream>

using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;
using namespace PoseWrapper;

namespace dfn_ci
{

Icp3D::Icp3D()
{
	parametersHelper.AddParameter<double>("GeneralParameters", "MaxCorrespondenceDistance", parameters.maxCorrespondenceDistance, DEFAULT_PARAMETERS.maxCorrespondenceDistance);
	parametersHelper.AddParameter<int>("GeneralParameters", "MaximumIterations", parameters.maximumIterations, DEFAULT_PARAMETERS.maximumIterations);
	parametersHelper.AddParameter<double>("GeneralParameters", "TransformationEpsilon", parameters.transformationEpsilon, DEFAULT_PARAMETERS.transformationEpsilon);
	parametersHelper.AddParameter<double>("GeneralParameters", "EuclideanFitnessEpsilon", parameters.euclideanFitnessEpsilon, DEFAULT_PARAMETERS.euclideanFitnessEpsilon);

	configurationFilePath = "";
	inUseGuess = false;
}

Icp3D::~Icp3D()
{
}

void Icp3D::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void Icp3D::process()
{
	if ( GetNumberOfPoints(inSourceCloud) == 0 || GetNumberOfPoints(inSinkCloud) == 0)
		{
		outSuccess = false;
		return;
		}
	VERIFY(!inUseGuess, "Icp3D Warning, a transformation guess was provided but this dfn Registration3D implementation does not use one, it will be ignored");

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputSourceCloud =
		pointCloudToPclPointCloudConverter.Convert(&inSourceCloud);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputSinkCloud =
		pointCloudToPclPointCloudConverter.Convert(&inSinkCloud);

	ValidateInputs(inputSourceCloud, inputSinkCloud);
	Transform3DConstPtr transform = ComputeTransform(inputSourceCloud, inputSinkCloud);
	outTransform = *transform;
}

const Icp3D::IcpOptionsSet Icp3D::DEFAULT_PARAMETERS =
	{
	.maxCorrespondenceDistance = 0.05,
	.maximumIterations = 50,
	.transformationEpsilon = 1e-8,
	.euclideanFitnessEpsilon = 1.0	
	};


/**
** This is the function that computes the detect the presence of the source point cloud within the sink point cloud, and computes the trasform that allows the source to be transformed to its position in
** the sink.
** Observe that in PCL terminology the sink cloud is called target cloud.
**
**/
Transform3DConstPtr Icp3D::ComputeTransform(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud)
	{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud (sourceCloud);
	icp.setInputTarget (sinkCloud);

	icp.setMaxCorrespondenceDistance (parameters.maxCorrespondenceDistance);
	icp.setMaximumIterations (parameters.maximumIterations);
	icp.setTransformationEpsilon (parameters.transformationEpsilon);
	icp.setEuclideanFitnessEpsilon (parameters.euclideanFitnessEpsilon);

	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align (*outputCloud);
	
	outSuccess = icp.hasConverged();
	if (outSuccess)
		{
		Eigen::Matrix4f eigenTransform = icp.getFinalTransformation();
		return eigenTransformToTransform3dConverter.Convert(eigenTransform);
		}
	else
		{
		Transform3DPtr transform = NewPose3D();
		return transform;
		}
	
	}

void Icp3D::ValidateParameters()
	{
	ASSERT( parameters.maxCorrespondenceDistance >= 0, "Icp3D Configuration error, Max Correspondence Distance is negative");
	ASSERT( parameters.maximumIterations >= 0, "Icp3D Configuration error, Maximum Iterations is negative");
	ASSERT( parameters.transformationEpsilon > 0, "Icp3D Configuration error, Transformation Epsilon is not strictly positive");
	ASSERT( parameters.euclideanFitnessEpsilon > 0, "Icp3D Configuration error, Euclidean Fitness Epsilon is not stricltly positive");
	}

void Icp3D::ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud)
	{
	ValidateCloud(sourceCloud);
	ValidateCloud(sinkCloud);
	}

void Icp3D::ValidateCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
	{
	for(unsigned pointIndex = 0; pointIndex < cloud->points.size(); pointIndex++)
		{
		const pcl::PointXYZ& point = cloud->points.at(pointIndex);
		ASSERT_EQUAL(point.x, point.x, "Icp3D Error, Cloud contains an NaN point");
		ASSERT_EQUAL(point.y, point.y, "Icp3D Error, Cloud contains an NaN point");
		ASSERT_EQUAL(point.z, point.z, "Icp3D Error, Cloud contains an NaN point");
		}
	}

}

/** @} */
