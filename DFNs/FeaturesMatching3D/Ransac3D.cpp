/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Ransac3D.cpp
 * @date 17/01/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the Ransac 3D class.
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Ransac3D.hpp"
#include <Errors/Assert.hpp>
#include <VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <EigenTransformToTransform3DConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/search.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <SupportTypes.hpp>

#include <stdlib.h>
#include <fstream>


using namespace Common;
using namespace Converters;
using namespace PoseWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace SupportTypes;

namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
Ransac3D::Ransac3D()
	{
	parameters.similarityThreshold = 0.9;
	parameters.inlierFraction = 0.25;
	parameters.correspondenceRandomness = 5;
	parameters.numberOfSamples = 3;
	parameters.maximumIterations = 50000;
	parameters.maxCorrespondenceDistance = 0.00125;

	parameters.ransacIterations = 50000;
	parameters.ransacOutlierRejectionThreshold = 0.05;
	parameters.transformationEpsilon = std::numeric_limits<double>::epsilon();
	parameters.euclideanFitnessEpsilon = std::numeric_limits<double>::epsilon();

	configurationFilePath = "";
	}

Ransac3D::~Ransac3D()
	{

	}

void Ransac3D::configure()
	{
	try
		{
		YAML::Node configuration= YAML::LoadFile( configurationFilePath );
		for(unsigned configuationIndex=0; configuationIndex < configuration.size(); configuationIndex++)
			{
			YAML::Node configurationNode = configuration[configuationIndex];
			Configure(configurationNode);
			}
		} 
	catch(YAML::ParserException& e) 
		{
    		ASSERT(false, e.what() );
		}
	}


void Ransac3D::process() 
	{
	PointCloudWithFeatures inputSourceCloud = 
		ConversionCache<VisualPointFeatureVector3DConstPtr, SupportTypes::PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Convert(inSourceFeaturesVector);
	PointCloudWithFeatures inputSinkCloud = 
		ConversionCache<VisualPointFeatureVector3DConstPtr, SupportTypes::PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Convert(inSinkFeaturesVector);
	ValidateInputs(inputSourceCloud, inputSinkCloud);
	outTransform = ComputeTransform(inputSourceCloud, inputSinkCloud);
	}


Transform3DConstPtr Ransac3D::ComputeTransform(Converters::SupportTypes::PointCloudWithFeatures sourceCloud, Converters::SupportTypes::PointCloudWithFeatures sinkCloud)
	{
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

	/*
	These parameters seems to not make any difference at all
	***
	ransac->setRANSACOutlierRejectionThreshold(parameters.ransacOutlierRejectionThreshold);
	ransac->setRANSACIterations(parameters.ransacIterations);
	ransac->setTransformationEpsilon(parameters.transformationEpsilon);
	ransac->setEuclideanFitnessEpsilon(parameters.euclideanFitnessEpsilon);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr sinkTree(new pcl::search::KdTree<pcl::PointXYZ>());
	sinkTree->setInputCloud(sinkCloud);
	ransac->setSearchMethodTarget(sinkTree);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr sourceTree(new pcl::search::KdTree<pcl::PointXYZ>());
	sourceTree->setInputCloud(sourceCloud);
	ransac->setSearchMethodSource(sourceTree);*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
	ransac->align(*outputCloud);
	
	outSuccess = ransac->hasConverged();
	if (outSuccess)
		{
		Eigen::Matrix4f eigenTransform = ransac->getFinalTransformation();
		PRINT_TO_LOG("Matrix ", eigenTransform);
		return ConversionCache<Eigen::Matrix4f, Transform3DConstPtr, EigenTransformToTransform3DConverter>::Convert(eigenTransform);
		}
	else
		{
		Transform3DPtr transform = new Transform3D();
		Reset(*transform);
		return transform;
		}
	
	}

PoseWrapper::Transform3DConstPtr Ransac3D::Convert(Eigen::Matrix4f eigenTransform)
	{
	Transform3DPtr transform = new Transform3D();	

	Eigen::Matrix3f eigenRotationMatrix = eigenTransform.block(0,0,3,3);
	Eigen::Quaternionf eigenRotation (eigenRotationMatrix);

	SetPosition(*transform, eigenTransform(0, 3), eigenTransform(1, 3), eigenTransform(2, 3) );
	SetOrientation(*transform, eigenRotation.x(), eigenRotation.y(), eigenRotation.z(), eigenRotation.w());

	return transform;
	}


void Ransac3D::ValidateParameters()
	{
	ASSERT( parameters.similarityThreshold >= 0, "Ransac3D Configuration error, similarity threshold is not positive");
	ASSERT( parameters.inlierFraction > 0, "Ransac3D Configuration error, inlier fraction is not stricltly positive");
	ASSERT( parameters.correspondenceRandomness >= 0, "Ransac3D Configuration error, correspondence randomness is not positive");
	ASSERT( parameters.numberOfSamples > 0, "Ransac3D Configuration error, number of samples is not stricltly positive");
	ASSERT( parameters.maximumIterations > 0, "Ransac3D Configuration error, number of iterations is not stricltly positive");
	ASSERT( parameters.maxCorrespondenceDistance >= 0, "Ransac3D Configuration error, max correspondence distance is not positive");

	ASSERT( parameters.ransacIterations > 0, "Ransac3D Configuration error, randac iterations is not stricltly positive");
	ASSERT( parameters.ransacOutlierRejectionThreshold >= 0, "Ransac3D Configuration error, ransac outlier rejection threshold is not positive");
	ASSERT( parameters.transformationEpsilon > 0, "Ransac3D Configuration error, transformation epsilon is not stricltly positive");
	ASSERT( parameters.euclideanFitnessEpsilon > 0, "Ransac3D Configuration error, euclidean fiteness epsilon is not stricltly positive");
	}

void Ransac3D::ValidateInputs(Converters::SupportTypes::PointCloudWithFeatures sourceCloud, Converters::SupportTypes::PointCloudWithFeatures sinkCloud)
	{
	ASSERT(sourceCloud.descriptorSize == sinkCloud.descriptorSize, "Ransac 3D Error, Source Cloud has different descriptor size than sink cloud");
	ValidateCloud(sourceCloud);
	ValidateCloud(sinkCloud);
	}

void Ransac3D::ValidateCloud(Converters::SupportTypes::PointCloudWithFeatures cloud)
	{
	ASSERT(cloud.pointCloud->points.size() == cloud.featureCloud->points.size(), "Ransac 3D Error, point cloud has different size than feature cloud");

	for(unsigned pointIndex = 0; pointIndex < cloud.pointCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = cloud.pointCloud->points.at(pointIndex);
		FeatureType feature = cloud.featureCloud->points.at(pointIndex);
		ASSERT(point.x == point.x, "Ransac 3D Error, Cloud contains an NaN point");
		ASSERT(point.y == point.y, "Ransac 3D Error, Cloud contains an NaN point");
		ASSERT(point.z == point.z, "Ransac 3D Error, Cloud contains an NaN point");
		for(unsigned componentIndex = 0; componentIndex < cloud.descriptorSize; componentIndex++)
			{
			ASSERT(feature.histogram[componentIndex] == feature.histogram[componentIndex], "Ransac 3D Error, Feature Cloud contains a NaN feature");
			}
		for(unsigned componentIndex = cloud.descriptorSize; componentIndex < MAX_FEATURES_NUMBER; componentIndex++)
			{
			ASSERT(feature.histogram[componentIndex] == 0, "Ransac 3D Error, Feature Cloud contains an invalid feature, probably it was a conversion error");
			}
		}
	}


void Ransac3D::Configure(const YAML::Node& configurationNode)
	{
	std::string nodeName = configurationNode["Name"].as<std::string>();
	if ( nodeName == "GeneralParameters")
		{
		parameters.similarityThreshold = configurationNode["SimilarityThreshold"].as<float>();
		parameters.inlierFraction = configurationNode["InlierFraction"].as<float>();
		parameters.correspondenceRandomness = configurationNode["CorrespondenceRandomness"].as<int>();
		parameters.numberOfSamples = configurationNode["NumberOfSamples"].as<int>();
		parameters.maximumIterations = configurationNode["MaximumIterations"].as<int>();
		parameters.maxCorrespondenceDistance = configurationNode["MaxCorrespondenceDistance"].as<float>();

		//parameters.ransacIterations = configurationNode["RansacIterations"].as<int>();
		//parameters.ransacOutlierRejectionThreshold = configurationNode["RansacOutlierRejectionThreshold"].as<float>();
		//parameters.transformationEpsilon = configurationNode["TransformationEpsilon"].as<double>();
		//parameters.euclideanFitnessEpsilon = configurationNode["EuclideanFitnessEpsilon"].as<double>();
		}
	//Ignore everything else
	}


}


/** @} */
