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
	PRINT_TO_LOG("Error", 0);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputSourceCloud = 
		ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(inSourceCloud);
	PRINT_TO_LOG("Error", 1);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputSinkCloud = 
		ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(inSinkCloud);
	PRINT_TO_LOG("Error", 2);
	ValidateInputs(inputSourceCloud, inputSinkCloud);
	PRINT_TO_LOG("Error", 3);
	outCorrespondenceMap = ComputeCorrespondences(inputSourceCloud, inputSinkCloud);
	PRINT_TO_LOG("Error", 4);
	//outFeaturesSet = ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Convert(harrisPoints);
	}


CorrespondenceMap3DConstPtr Ransac3D::ComputeCorrespondences(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud)
	{
	CorrespondenceMap3DPtr correspondenceMap = new CorrespondenceMap3D();

	pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTreePtr(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(sourceCloud));
	model->setInputCloud(sourceCloud);	
	model->setInputTarget(sinkCloud); 
	model->setSamplesMaxDist(0.01, searchTreePtr);

	pcl::RandomSampleConsensus<pcl::PointXYZ>::Ptr ransac(new pcl::RandomSampleConsensus<pcl::PointXYZ>(model));
	ransac->setMaxIterations(parameters.maxIterationsNumber);
	ransac->setProbability(parameters.outliersFreeProbability);
	ransac->setDistanceThreshold(parameters.distanceThreshold);

	std::vector<int> inliersSet;
	ransac->computeModel();
	ransac->getInliers(inliersSet);

	return correspondenceMap;
	}


void Ransac3D::ValidateParameters()
	{
	ASSERT( parameters.maxIterationsNumber > 0, "Ransac3D Configuration error,max iteration number is not strictly positive");
	ASSERT( parameters.outliersFreeProbability >= 0 && parameters.outliersFreeProbability <= 1, "Ransac3D Configuration error, outlier free probability is not in set [0,1]");
	ASSERT( parameters.distanceThreshold >= 0, "Ransac3D Configuration error, distance threshold is negative");
	}

void Ransac3D::ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud)
	{

	}


void Ransac3D::Configure(const YAML::Node& configurationNode)
	{
	std::string nodeName = configurationNode["Name"].as<std::string>();
	if ( nodeName == "GeneralParameters")
		{
		parameters.maxIterationsNumber = configurationNode["MaxIterationsNumber"].as<int>();
		parameters.outliersFreeProbability = configurationNode["OutliersFreeProbability"].as<float>();
		parameters.distanceThreshold = configurationNode["DistanceThreshold"].as<float>();
		}
	//Ignore everything else
	}


}


/** @} */
