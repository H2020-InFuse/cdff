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

#include <stdlib.h>
#include <fstream>


using namespace Common;
using namespace Converters;
using namespace CorrespondenceMap3DWrapper;
using namespace PointCloudWrapper;

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
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputSourceCloud = 
		ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(inSourceCloud);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputSinkCloud = 
		ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(inSinkCloud);
	ValidateInputs(inputSourceCloud, inputSinkCloud);
	outCorrespondenceMap = ComputeCorrespondences(inputSourceCloud, inputSinkCloud);
	//outFeaturesSet = ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Convert(harrisPoints);
	}


CorrespondenceMap3DConstPtr Ransac3D::ComputeCorrespondences(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud)
	{
	CorrespondenceMap3DPtr correspondenceMap = new CorrespondenceMap3D();

	pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTreePtr(new pcl::search::KdTree<pcl::PointXYZ>);
	searchTreePtr->setInputCloud(sinkCloud);
	searchTreePtr->setEpsilon(std::numeric_limits<double>::epsilon());
	pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(sourceCloud));
	model->setInputTarget(sinkCloud); 
	model->setSamplesMaxDist(parameters.samplesMaxDistance, searchTreePtr);

	pcl::RandomSampleConsensus<pcl::PointXYZ>::Ptr ransac(new pcl::RandomSampleConsensus<pcl::PointXYZ>(model));
	ransac->setMaxIterations(parameters.maxIterationsNumber);
	ransac->setProbability(parameters.outliersFreeProbability);
	ransac->setDistanceThreshold(parameters.distanceThreshold);

	std::vector<int> inliersSet;
	ransac->computeModel();
	ransac->getInliers(inliersSet);

	for(unsigned inlierIndex = 0; inlierIndex < inliersSet.size(); inlierIndex++)
		{	
		pcl::PointXYZ sourcePoint = sinkCloud->points.at(inliersSet.at(inlierIndex));
		BaseTypesWrapper::Point3D sourcePoint3D;
		sourcePoint3D.x = sourcePoint.x;
		sourcePoint3D.y = sourcePoint.y;
		sourcePoint3D.z = sourcePoint.z;
		BaseTypesWrapper::Point3D sinkPoint3D;
		sinkPoint3D.x = sourcePoint.x;
		sinkPoint3D.y = sourcePoint.y;
		sinkPoint3D.z = sourcePoint.z;
		AddCorrespondence(*correspondenceMap, sourcePoint3D, sinkPoint3D, 1.0);
		}

	return correspondenceMap;
	}


void Ransac3D::ValidateParameters()
	{
	ASSERT( parameters.maxIterationsNumber > 0, "Ransac3D Configuration error,max iteration number is not strictly positive");
	ASSERT( parameters.outliersFreeProbability >= 0 && parameters.outliersFreeProbability <= 1, "Ransac3D Configuration error, outlier free probability is not in set [0,1]");
	ASSERT( parameters.distanceThreshold >= 0, "Ransac3D Configuration error, distance threshold is negative");
	ASSERT( parameters.samplesMaxDistance >= 0, "Ransac3D Configuration error, samples max distance is negative");
	}

void Ransac3D::ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud)
	{
	for(unsigned pointIndex = 0; pointIndex < sourceCloud->size(); pointIndex++)
		{
		ASSERT(sourceCloud->points.at(pointIndex).x == sourceCloud->points.at(pointIndex).x, "Ransac 3D Error, Source Cloud contains an NaN point");
		ASSERT(sourceCloud->points.at(pointIndex).y == sourceCloud->points.at(pointIndex).y, "Ransac 3D Error, Source Cloud contains an NaN point");
		ASSERT(sourceCloud->points.at(pointIndex).z == sourceCloud->points.at(pointIndex).z, "Ransac 3D Error, Source Cloud contains an NaN point");
		}
	for(unsigned pointIndex = 0; pointIndex < sinkCloud->size(); pointIndex++)
		{
		ASSERT(sinkCloud->points.at(pointIndex).x == sinkCloud->points.at(pointIndex).x, "Ransac 3D Error, Sink Cloud contains an NaN point");
		ASSERT(sinkCloud->points.at(pointIndex).y == sinkCloud->points.at(pointIndex).y, "Ransac 3D Error, Sink Cloud contains an NaN point");
		ASSERT(sinkCloud->points.at(pointIndex).z == sinkCloud->points.at(pointIndex).z, "Ransac 3D Error, Sink Cloud contains an NaN point");
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
