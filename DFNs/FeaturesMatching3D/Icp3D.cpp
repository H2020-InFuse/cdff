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
		
	//Input Validation
	ValidateInputs();

	// Read data from input ports
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputSourceCloud = Convert(inSourceFeatures);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputSinkCloud = Convert(inSinkFeatures);

	// Process data
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

pcl::PointCloud<pcl::PointXYZ>::Ptr Icp3D::Convert(const VisualPointFeatureVector3D& features)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	int numberOfPoints = GetNumberOfPoints(features);
	cloud->points.resize(numberOfPoints);
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		cloud->points.at(pointIndex).x = GetXCoordinate(features, pointIndex);
		cloud->points.at(pointIndex).y = GetYCoordinate(features, pointIndex);
		cloud->points.at(pointIndex).z = GetZCoordinate(features, pointIndex);
		}
	return cloud;
	}

Pose3DConstPtr Icp3D::InverseConvert(Eigen::Matrix4f eigenTransformSceneInModel)
{
	Pose3DConstPtr scenePoseInModel = EigenTransformToTransform3DConverter().Convert(eigenTransformSceneInModel);

	Pose3DPtr modelPoseInScene = NewPose3D();
	SetPosition(*modelPoseInScene, -GetXPosition(*scenePoseInModel), -GetYPosition(*scenePoseInModel), -GetZPosition(*scenePoseInModel));
	SetOrientation(*modelPoseInScene, -GetXOrientation(*scenePoseInModel), -GetYOrientation(*scenePoseInModel), -GetZOrientation(*scenePoseInModel), GetWOrientation(*scenePoseInModel));
	delete(scenePoseInModel);
	return modelPoseInScene;
}

/**
 * This function detects if the source pointcloud is within the sink pointcloud,
 * and computes the geometric transformation that turns the source cloud into
 * its position in the sink cloud.
 *
 * Observe that in PCL terminology, the sink cloud is called target cloud.
 */
Pose3DConstPtr Icp3D::ComputeTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr sinkCloud)
{
	// Setup PCL's ICP algorithm
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(sourceCloud);
	icp.setInputTarget(sinkCloud);

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
		Eigen::Matrix4f eigenTransformSceneInModel = icp.getFinalTransformation();
		return InverseConvert(eigenTransformSceneInModel);
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

void Icp3D::ValidateInputs()
{
	VisualPointFeature3DType featureType = GetFeatureType(inSourceFeatures);
	ASSERT(featureType == GetFeatureType(inSinkFeatures), "Ransac3D Error, source and sink image do not have same type");
	ValidateCloud(inSourceFeatures);
	ValidateCloud(inSinkFeatures);
}

void Icp3D::ValidateCloud(const VisualPointFeatureVector3D& features)
{
	int numberOfPoints = GetNumberOfPoints(features);
	if (numberOfPoints == 0)
		{
		return;
		}

	int baseDescriptorSize = GetNumberOfDescriptorComponents(features, 0);
	for (unsigned pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
	{
		float x = GetXCoordinate(features, pointIndex);
		float y = GetYCoordinate(features, pointIndex);
		float z = GetZCoordinate(features, pointIndex);
		ASSERT(x == x, "BestDescriptorMatch Error, cloud contains a NaN point");
		ASSERT(y == y, "BestDescriptorMatch Error, cloud contains a NaN point");
		ASSERT(z == z, "BestDescriptorMatch Error, cloud contains a NaN point");

		int descriptorSize = GetNumberOfDescriptorComponents(features, pointIndex);
		ASSERT(descriptorSize == baseDescriptorSize, "Ransac3D Error, Descriptor sizes mismatch");

		for (unsigned componentIndex = 0; componentIndex < descriptorSize; componentIndex++)
		{
			float component = GetDescriptorComponent(features, pointIndex, componentIndex);
			ASSERT(component == component, "Ransac3D Error, feature cloud contains a NaN feature");
		}
	}
}

}
}
}

/** @} */
