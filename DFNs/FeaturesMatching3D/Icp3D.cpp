/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "Icp3D.hpp"

#include <Converters/EigenTransformToTransform3DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/registration/icp.h>
#include <yaml-cpp/yaml.h>

using namespace Converters;
using namespace PoseWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace SupportTypes;

namespace CDFF
{
namespace DFN
{
namespace FeaturesMatching3D
{

Icp3D::Icp3D()
{
	parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<double>(
		"GeneralParameters", "MaxCorrespondenceDistance",
		parameters.maxCorrespondenceDistance, DEFAULT_PARAMETERS.maxCorrespondenceDistance);
	parametersHelper.AddParameter<int>(
		"GeneralParameters", "MaximumIterations",
		parameters.maximumIterations, DEFAULT_PARAMETERS.maximumIterations);
	parametersHelper.AddParameter<double>(
		"GeneralParameters", "TransformationEpsilon",
		parameters.transformationEpsilon, DEFAULT_PARAMETERS.transformationEpsilon);
	parametersHelper.AddParameter<double>(
		"GeneralParameters", "EuclideanFitnessEpsilon",
		parameters.euclideanFitnessEpsilon, DEFAULT_PARAMETERS.euclideanFitnessEpsilon);

	configurationFilePath = "";
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
	// Handle empty keypoint vectors
	if (GetNumberOfPoints(inSourceFeatures) == 0 || GetNumberOfPoints(inSinkFeatures) == 0)
	{
		outSuccess = false;
		Reset(outTransform);
		return;
	}

	// Read data from input ports
	PointCloudWithFeatures inputSourceCloud =
		visualPointFeatureVector3DToPclPointCloud.Convert(&inSourceFeatures);
	PointCloudWithFeatures inputSinkCloud =
		visualPointFeatureVector3DToPclPointCloud.Convert(&inSinkFeatures);

	// Process data
	ValidateInputs(inputSourceCloud, inputSinkCloud);
	Pose3DConstPtr tmp = ComputeTransform(inputSourceCloud, inputSinkCloud);

	// Write data to output port
	Copy(*tmp, outTransform);
	delete tmp;
}

const Icp3D::IcpOptionsSet Icp3D::DEFAULT_PARAMETERS =
{
	/*.maxCorrespondenceDistance =*/ 0.05,
	/*.maximumIterations =*/ 50,
	/*.transformationEpsilon =*/ 1e-8,
	/*.euclideanFitnessEpsilon =*/ 1.0
};

Pose3DConstPtr Icp3D::Convert(Eigen::Matrix4f eigenTransform)
{
	Pose3DPtr transform = new Pose3D;

	Eigen::Matrix3f eigenRotationMatrix = eigenTransform.block(0,0,3,3);
	Eigen::Quaternionf eigenRotation(eigenRotationMatrix);

	SetPosition(*transform, eigenTransform(0, 3), eigenTransform(1, 3), eigenTransform(2, 3) );
	SetOrientation(*transform, eigenRotation.x(), eigenRotation.y(), eigenRotation.z(), eigenRotation.w());

	return transform;
}

/**
 * This function detects if the source pointcloud is within the sink pointcloud,
 * and computes the geometric transformation that turns the source cloud into
 * its position in the sink cloud.
 *
 * Observe that in PCL terminology, the sink cloud is called target cloud.
 */
Pose3DConstPtr Icp3D::ComputeTransform(PointCloudWithFeatures sourceCloud, PointCloudWithFeatures sinkCloud)
{
	// Setup PCL's ICP algorithm
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(sourceCloud.pointCloud);
	icp.setInputTarget(sinkCloud.pointCloud);

	icp.setMaxCorrespondenceDistance(parameters.maxCorrespondenceDistance);
	icp.setMaximumIterations(parameters.maximumIterations);
	icp.setTransformationEpsilon(parameters.transformationEpsilon);
	icp.setEuclideanFitnessEpsilon(parameters.euclideanFitnessEpsilon);

	// Setup output
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Run ICP
	icp.align(*outputCloud);

	// Check convergence
	outSuccess = icp.hasConverged();
	if (outSuccess)
	{
		Eigen::Matrix4f eigenTransform = icp.getFinalTransformation();
		return EigenTransformToTransform3DConverter().Convert(eigenTransform);
	}
	else
	{
		Pose3DPtr transform = new Pose3D;
		Reset(*transform);
		return transform;
	}
}

void Icp3D::ValidateParameters()
{
	ASSERT(parameters.maxCorrespondenceDistance >= 0, "Icp3D Configuration error, Max Correspondence Distance is negative");
	ASSERT(parameters.maximumIterations >= 0, "Icp3D Configuration error, Maximum Iterations is negative");
	ASSERT(parameters.transformationEpsilon > 0, "Icp3D Configuration error, Transformation Epsilon is not strictly positive");
	ASSERT(parameters.euclideanFitnessEpsilon > 0, "Icp3D Configuration error, Euclidean Fitness Epsilon is not strictly positive");
}

void Icp3D::ValidateInputs(PointCloudWithFeatures sourceCloud, PointCloudWithFeatures sinkCloud)
{
	ASSERT(sourceCloud.descriptorSize == sinkCloud.descriptorSize, "Icp3D Error, source cloud and sink cloud have a different descriptor size");
	ValidateCloud(sourceCloud);
	ValidateCloud(sinkCloud);
}

void Icp3D::ValidateCloud(PointCloudWithFeatures cloud)
{
	ASSERT(cloud.pointCloud->points.size() == cloud.featureCloud->points.size(), "Icp3D Error, point cloud and feature cloud don't have the same size");

	for (unsigned pointIndex = 0; pointIndex < cloud.pointCloud->points.size(); pointIndex++)
	{
		pcl::PointXYZ point = cloud.pointCloud->points.at(pointIndex);
		FeatureType feature = cloud.featureCloud->points.at(pointIndex);
		ASSERT(point.x == point.x, "Icp3D Error, cloud contains a NaN point");
		ASSERT(point.y == point.y, "Icp3D Error, cloud contains a NaN point");
		ASSERT(point.z == point.z, "Icp3D Error, cloud contains a NaN point");
		for (unsigned componentIndex = 0; componentIndex < cloud.descriptorSize; componentIndex++)
		{
			ASSERT(feature.histogram[componentIndex] == feature.histogram[componentIndex], "Icp3D Error, feature cloud contains a NaN feature");
		}
		for (unsigned componentIndex = cloud.descriptorSize; componentIndex < MAX_FEATURES_NUMBER; componentIndex++)
		{
			ASSERT(feature.histogram[componentIndex] == 0, "Icp3D Error, feature cloud contains an invalid feature (probably coming from a conversion error)");
		}
	}
}

}
}
}

/** @} */
