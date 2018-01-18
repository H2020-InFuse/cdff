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
#include <PointCloudToPclPointCloudConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/search.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <stdlib.h>
#include <fstream>


using namespace Common;
using namespace Converters;
using namespace PoseWrapper;
using namespace VisualPointFeatureVector3DWrapper;

namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
Ransac3D::Ransac3D()
	{
	parameters.maxIterationsNumber = 1000;
	parameters.outliersFreeProbability = 0.99;
	parameters.distanceThreshold = 0.01;
	parameters.samplesMaxDistance = 0.01;

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
	//pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputSourceCloud = 
	//	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(inSourceCloud);
	//pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputSinkCloud = 
	//	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(inSinkCloud);
	ValidateInputs(inSourceFeaturesVector, inSinkFeaturesVector);
	outTransform = ComputeTransform(inSourceFeaturesVector, inSinkFeaturesVector);
	//outFeaturesSet = ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Convert(harrisPoints);
	}


Transform3DConstPtr Ransac3D::ComputeTransform(VisualPointFeatureVector3DConstPtr sourceFeaturesVector, VisualPointFeatureVector3DConstPtr sinkFeaturesVector)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>());
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*sourceFeaturesVector); pointIndex++)
		{
		pcl::PointXYZ sourcePoint( GetXCoordinate(*sourceFeaturesVector, pointIndex), GetYCoordinate(*sourceFeaturesVector, pointIndex), GetZCoordinate(*sourceFeaturesVector, pointIndex) );
		sourceCloud->points.push_back(sourcePoint);
		}
	pcl::PointCloud<pcl::PointXYZ>::Ptr sinkCloud(new pcl::PointCloud<pcl::PointXYZ>());
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*sinkFeaturesVector); pointIndex++)
		{
		pcl::PointXYZ sinkPoint( GetXCoordinate(*sinkFeaturesVector, pointIndex), GetYCoordinate(*sinkFeaturesVector, pointIndex), GetZCoordinate(*sinkFeaturesVector, pointIndex) );
		sinkCloud->points.push_back(sinkPoint);
		}

	const unsigned MAX_FEATURES_SIZE = 10;
	typedef pcl::Histogram<MAX_FEATURES_SIZE> FeatureT;	
	pcl::PointCloud<FeatureT>::Ptr sourceFeaturesCloud(new pcl::PointCloud<FeatureT>() );
	for(unsigned pointIndex = 0; pointIndex < GetNumberOfPoints(*sourceFeaturesVector); pointIndex++)
		{
		FeatureT feature;
		for(unsigned componentIndex = 0; componentIndex < MAX_FEATURES_SIZE; componentIndex++)
			{
			if (componentIndex < GetNumberOfDescriptorComponents(*sourceFeaturesVector, pointIndex) )
				{
				feature.histogram[componentIndex] = GetDescriptorComponent(*sourceFeaturesVector, pointIndex, componentIndex);
				}
			else
				{
				feature.histogram[componentIndex] = 0;
				}
			}
		sourceFeaturesCloud->points.push_back(feature);
		}
	pcl::PointCloud<FeatureT>::Ptr sinkFeaturesCloud(new pcl::PointCloud<FeatureT>() );
	for(unsigned pointIndex = 0; pointIndex < GetNumberOfPoints(*sinkFeaturesVector); pointIndex++)
		{
		FeatureT feature;
		for(unsigned componentIndex = 0; componentIndex < MAX_FEATURES_SIZE; componentIndex++)
			{
			if (componentIndex < GetNumberOfDescriptorComponents(*sinkFeaturesVector, pointIndex) )
				{
				feature.histogram[componentIndex] = GetDescriptorComponent(*sinkFeaturesVector, pointIndex, componentIndex);
				}
			else
				{
				feature.histogram[componentIndex] = 0;
				}
			}
		sinkFeaturesCloud->points.push_back(feature);
		}


	pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, FeatureT>::Ptr ransac(new pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, FeatureT>);
	ransac->setSourceFeatures(sourceFeaturesCloud);
	ransac->setTargetFeatures(sinkFeaturesCloud);
	ransac->setInputSource(sourceCloud);
	ransac->setInputTarget(sinkCloud);	
	ransac->setSimilarityThreshold(0.01);
	ransac->setInlierFraction(0.50);
	ransac->setCorrespondenceRandomness(3);
	ransac->setNumberOfSamples(10);
	ransac->setMaximumIterations(1000);
	ransac->setRANSACIterations(1000);
	ransac->setRANSACOutlierRejectionThreshold(0.99);
	ransac->setMaxCorrespondenceDistance(0.1);
	ransac->setTransformationEpsilon(0.01);
	ransac->setEuclideanFitnessEpsilon(0.01);

	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
	ransac->align(*outputCloud);
	Eigen::Matrix4f eigenTransform = ransac->getFinalTransformation();

	Transform3DPtr transform = new Transform3D();
	SetPosition(*transform, eigenTransform(0, 3), eigenTransform(1, 3), eigenTransform(2, 3) );
	float quaternionW = std::sqrt(1.0 + eigenTransform(0, 0) + eigenTransform(1, 1) + eigenTransform(2, 2)) / 2.0;
	float quaternionX = (eigenTransform(2, 1) - eigenTransform(1, 2)) / (4 * quaternionW);
	float quaternionY = (eigenTransform(0, 2) - eigenTransform(2, 0)) / (4 * quaternionW);
	float quaternionZ = (eigenTransform(1, 0) - eigenTransform(0, 1)) / (4 * quaternionW);
	SetOrientation(*transform, quaternionX, quaternionY, quaternionZ, quaternionW);

	return transform;
	}


void Ransac3D::ValidateParameters()
	{
	ASSERT( parameters.maxIterationsNumber > 0, "Ransac3D Configuration error,max iteration number is not strictly positive");
	ASSERT( parameters.outliersFreeProbability >= 0 && parameters.outliersFreeProbability <= 1, "Ransac3D Configuration error, outlier free probability is not in set [0,1]");
	ASSERT( parameters.distanceThreshold >= 0, "Ransac3D Configuration error, distance threshold is negative");
	ASSERT( parameters.samplesMaxDistance >= 0, "Ransac3D Configuration error, samples max distance is negative");
	}

void Ransac3D::ValidateInputs(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr source, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sink)
	{
	for(unsigned pointIndex = 0; pointIndex < GetNumberOfPoints(*source); pointIndex++)
		{
		ASSERT(GetXCoordinate(*source, pointIndex) == GetXCoordinate(*source, pointIndex), "Ransac 3D Error, Source Cloud contains an NaN point");
		ASSERT(GetYCoordinate(*source, pointIndex) == GetYCoordinate(*source, pointIndex), "Ransac 3D Error, Source Cloud contains an NaN point");
		ASSERT(GetZCoordinate(*source, pointIndex) == GetZCoordinate(*source, pointIndex), "Ransac 3D Error, Source Cloud contains an NaN point");
		}
	for(unsigned pointIndex = 0; pointIndex < GetNumberOfPoints(*sink); pointIndex++)
		{
		ASSERT(GetXCoordinate(*sink, pointIndex) == GetXCoordinate(*sink, pointIndex), "Ransac 3D Error, Sink Cloud contains an NaN point");
		ASSERT(GetYCoordinate(*sink, pointIndex) == GetYCoordinate(*sink, pointIndex), "Ransac 3D Error, Sink Cloud contains an NaN point");
		ASSERT(GetZCoordinate(*sink, pointIndex) == GetZCoordinate(*sink, pointIndex), "Ransac 3D Error, Sink Cloud contains an NaN point");
		}
	}


void Ransac3D::Configure(const YAML::Node& configurationNode)
	{
	std::string nodeName = configurationNode["Name"].as<std::string>();
	if ( nodeName == "GeneralParameters")
		{
		parameters.maxIterationsNumber = configurationNode["MaxIterationsNumber"].as<int>();
		parameters.outliersFreeProbability = configurationNode["OutliersFreeProbability"].as<float>();
		parameters.distanceThreshold = configurationNode["DistanceThreshold"].as<float>();
		parameters.samplesMaxDistance = configurationNode["SamplesMaxDistance"].as<float>();
		}
	//Ignore everything else
	}


}


/** @} */
