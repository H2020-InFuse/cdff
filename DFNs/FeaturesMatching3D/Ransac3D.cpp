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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
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

const Ransac3D::RansacOptionsSet Ransac3D::DEFAULT_PARAMETERS =
{
	/*.similarityThreshold =*/ 0.9,
	/*.inlierFraction =*/ 0.25,
	/*.correspondenceRandomness =*/ 5,
	/*.numberOfSamples =*/ 3,
	/*.maximumIterations =*/ 50000,
	/*.maxCorrespondenceDistance =*/ 0.00125
};

/**
 * This function detects if the source pointcloud is within the sink pointcloud,
 * and computes the geometric transformation that turns the source cloud into
 * its position in the sink cloud.
 *
 * Observe that in PCL terminology, the sink cloud is called target cloud.
 *
 * Note: a few more parameters used to be given for SampleConsensusPrerejective:
 * RANSACOutlierRejectionThreshold, RANSACIterations, TransformationEpsilon,
 * EuclideanFitnessEpislon, SearchMethodTarget, and SearchMethodSource. However,
 * besides the search method, those parameters don't seem to be used by the
 * algorithm. The set method is set in a standard way in the algorithm.
 */
Pose3DConstPtr Ransac3D::ComputeTransform(PointCloudWithFeatures sourceCloud, PointCloudWithFeatures sinkCloud)
{
	// Setup PCL's RANSAC algorithm
	pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, FeatureType>::Ptr ransac(new pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, FeatureType>);
	ransac->setSourceFeatures(sourceCloud.featureCloud);
	ransac->setTargetFeatures(sinkCloud.featureCloud);
	ransac->setInputSource(sourceCloud.pointCloud);
	ransac->setInputTarget(sinkCloud.pointCloud);

	ransac->setSimilarityThreshold(parameters.similarityThreshold);
	ransac->setInlierFraction(parameters.inlierFraction);
	ransac->setCorrespondenceRandomness(parameters.correspondenceRandomness);
	ransac->setNumberOfSamples(parameters.numberOfSamples);
	ransac->setMaximumIterations(parameters.maximumIterations);
	ransac->setMaxCorrespondenceDistance(parameters.maxCorrespondenceDistance);

	// Setup output
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);

	try 
		{
		// Run RANSAC
		ransac->align(*outputCloud);

		// Check convergence
		outSuccess = ransac->hasConverged();
		}
	catch ( ... )
		{
		VERIFY(false, "Ransac failed with exception!");
		outSuccess = false;
		}

	if (outSuccess)
	{
		Eigen::Matrix4f eigenTransformSceneInModel = ransac->getFinalTransformation();
		Eigen::Matrix4f eigenTransformModelInScene = eigenTransformSceneInModel.inverse();
		return EigenTransformToTransform3DConverter().Convert(eigenTransformModelInScene);
	}
	else
	{
		Pose3DPtr transform = new Pose3D;
		Reset(*transform);
		return transform;
	}
}

Pose3DConstPtr Ransac3D::Convert(Eigen::Matrix4f eigenTransform)
{
	Pose3DPtr transform = new Pose3D;

	Eigen::Matrix3f eigenRotationMatrix = eigenTransform.block(0,0,3,3);
	Eigen::Quaternionf eigenRotation(eigenRotationMatrix);

	SetPosition(*transform, eigenTransform(0, 3), eigenTransform(1, 3), eigenTransform(2, 3) );
	SetOrientation(*transform, eigenRotation.x(), eigenRotation.y(), eigenRotation.z(), eigenRotation.w());

	return transform;
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

void Ransac3D::ValidateInputs(PointCloudWithFeatures sourceCloud, PointCloudWithFeatures sinkCloud)
{
	ASSERT(sourceCloud.descriptorSize == sinkCloud.descriptorSize, "Ransac3D Error, source cloud and sink cloud have a different descriptor size");
	ValidateCloud(sourceCloud);
	ValidateCloud(sinkCloud);
}

void Ransac3D::ValidateCloud(PointCloudWithFeatures cloud)
{
	ASSERT(cloud.pointCloud->points.size() == cloud.featureCloud->points.size(), "Ransac3D Error, point cloud and feature cloud don't have the same size");

	for (unsigned pointIndex = 0; pointIndex < cloud.pointCloud->points.size(); pointIndex++)
	{
		pcl::PointXYZ point = cloud.pointCloud->points.at(pointIndex);
		FeatureType feature = cloud.featureCloud->points.at(pointIndex);
		ASSERT(point.x == point.x, "Ransac3D Error, cloud contains a NaN point");
		ASSERT(point.y == point.y, "Ransac3D Error, cloud contains a NaN point");
		ASSERT(point.z == point.z, "Ransac3D Error, cloud contains a NaN point");
		for (unsigned componentIndex = 0; componentIndex < cloud.descriptorSize; componentIndex++)
		{
			ASSERT(feature.histogram[componentIndex] == feature.histogram[componentIndex], "Ransac3D Error, feature cloud contains a NaN feature");
		}
		for (unsigned componentIndex = cloud.descriptorSize; componentIndex < MAX_FEATURES_NUMBER; componentIndex++)
		{
			ASSERT(feature.histogram[componentIndex] == 0, "Ransac3D Error, feature cloud contains an invalid feature (probably coming from a conversion error)");
		}
	}
}

}
}
}

/** @} */
