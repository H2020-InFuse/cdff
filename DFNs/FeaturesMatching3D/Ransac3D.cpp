/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "Ransac3D.hpp"

#include <Converters/EigenTransformToTransform3DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

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

Ransac3D::Ransac3D()
{
	parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<float>(
		"GeneralParameters", "SimilarityThreshold",
		parameters.similarityThreshold, DEFAULT_PARAMETERS.similarityThreshold);
	parametersHelper.AddParameter<float>(
		"GeneralParameters", "InlierFraction",
		parameters.inlierFraction, DEFAULT_PARAMETERS.inlierFraction);
	parametersHelper.AddParameter<int>(
		"GeneralParameters", "CorrespondenceRandomness",
		parameters.correspondenceRandomness, DEFAULT_PARAMETERS.correspondenceRandomness);
	parametersHelper.AddParameter<int>(
		"GeneralParameters", "NumberOfSamples",
		parameters.numberOfSamples, DEFAULT_PARAMETERS.numberOfSamples);
	parametersHelper.AddParameter<int>(
		"GeneralParameters", "MaximumIterations",
		parameters.maximumIterations, DEFAULT_PARAMETERS.maximumIterations);
	parametersHelper.AddParameter<float>(
		"GeneralParameters", "MaxCorrespondenceDistance",
		parameters.maxCorrespondenceDistance, DEFAULT_PARAMETERS.maxCorrespondenceDistance);

	configurationFilePath = "";
}

Ransac3D::~Ransac3D()
{
}

void Ransac3D::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void Ransac3D::process()
{
	// Handle empty keypoint vectors
	if (GetNumberOfPoints(inSourceFeatures) == 0 || GetNumberOfPoints(inSinkFeatures) == 0)
	{
		outSuccess = false;
		Pose3D transform;
		Reset(transform);
		outTransform = transform;
		return;
	}

	//Input Validation
	ValidateInputs();

	// Read data from input ports
	VisualPointFeature3DType featureType = GetFeatureType(inSourceFeatures);
	switch(featureType)
		{
		case SHOT_DESCRIPTOR: 
			ProcessOnShotDescriptors();
			break;
		case PFH_DESCRIPTOR:
			ProcessOnPfhDescriptors();
			break;
		default:
			ASSERT(false, "Ransac3D error, unhandled feature type in input");
		}
}

void Ransac3D::ProcessOnShotDescriptors()
	{
	PointCloudWithFeatures<pcl::SHOT352> inputSourceCloud =
		visualPointFeatureVector3DToPclPointCloud.Convert<pcl::SHOT352>(&inSourceFeatures);
	PointCloudWithFeatures<pcl::SHOT352> inputSinkCloud =
		visualPointFeatureVector3DToPclPointCloud.Convert<pcl::SHOT352>(&inSinkFeatures);

	// Process data
	Pose3DConstPtr tmp = ComputeTransform(inputSourceCloud, inputSinkCloud);

	// Write data to output port
	Copy(*tmp, outTransform);
	delete tmp;
	}

void Ransac3D::ProcessOnPfhDescriptors()
	{
	PointCloudWithFeatures<pcl::PFHSignature125> inputSourceCloud =
		visualPointFeatureVector3DToPclPointCloud.Convert<pcl::PFHSignature125>(&inSourceFeatures);
	PointCloudWithFeatures<pcl::PFHSignature125> inputSinkCloud =
		visualPointFeatureVector3DToPclPointCloud.Convert<pcl::PFHSignature125>(&inSinkFeatures);

	// Process data
	Pose3DConstPtr tmp = ComputeTransform(inputSourceCloud, inputSinkCloud);

	// Write data to output port
	Copy(*tmp, outTransform);
	delete tmp;
	}

const Ransac3D::RansacOptionsSet Ransac3D::DEFAULT_PARAMETERS =
{
	/*.similarityThreshold =*/ 0.9,
	/*.inlierFraction =*/ 0.25,
	/*.correspondenceRandomness =*/ 5,
	/*.numberOfSamples =*/ 3,
	/*.maximumIterations =*/ 50000,
	/*.maxCorrespondenceDistance =*/ 0.00125
};

Pose3DConstPtr Ransac3D::InverseConvert(Eigen::Matrix4f eigenTransformSceneInModel)
{
	Pose3DConstPtr scenePoseInModel = EigenTransformToTransform3DConverter().Convert(eigenTransformSceneInModel);

	Pose3DPtr modelPoseInScene = NewPose3D();
	SetPosition(*modelPoseInScene, -GetXPosition(*scenePoseInModel), -GetYPosition(*scenePoseInModel), -GetZPosition(*scenePoseInModel));
	SetOrientation(*modelPoseInScene, -GetXOrientation(*scenePoseInModel), -GetYOrientation(*scenePoseInModel), -GetZOrientation(*scenePoseInModel), GetWOrientation(*scenePoseInModel));
	delete(scenePoseInModel);
	return modelPoseInScene;
}

void Ransac3D::ValidateParameters()
{
	ASSERT(parameters.similarityThreshold >= 0, "Ransac3D Configuration error, similarity threshold is not positive");
	ASSERT(parameters.inlierFraction > 0, "Ransac3D Configuration error, inlier fraction is not stricltly positive");
	ASSERT(parameters.correspondenceRandomness >= 0, "Ransac3D Configuration error, correspondence randomness is not positive");
	ASSERT(parameters.numberOfSamples > 0, "Ransac3D Configuration error, number of samples is not strictly positive");
	ASSERT(parameters.maximumIterations > 0, "Ransac3D Configuration error, number of iterations is not strictly positive");
	ASSERT(parameters.maxCorrespondenceDistance >= 0, "Ransac3D Configuration error, max correspondence distance is not positive");
}

void Ransac3D::ValidateInputs()
{
	VisualPointFeature3DType featureType = GetFeatureType(inSourceFeatures);
	ASSERT(featureType == GetFeatureType(inSinkFeatures), "Ransac3D Error, source and sink image do not have same type");
	ValidateCloud(inSourceFeatures);
	ValidateCloud(inSinkFeatures);
}

void Ransac3D::ValidateCloud(const VisualPointFeatureVector3D& features)
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
