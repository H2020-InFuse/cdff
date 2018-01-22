/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisDetector3D.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the Harris Detector 3D class.
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
#include "HarrisDetector3D.hpp"
#include <Errors/Assert.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>


#include <stdlib.h>
#include <fstream>


using namespace Common;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;

namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
HarrisDetector3D::HarrisDetector3D()
	{
	parameters.nonMaxSuppression = true;
	parameters.radius = 0.01;
	parameters.searchRadius = 0.01;
	parameters.detectionThreshold = 0;
	parameters.enableRefinement = false;
	parameters.numberOfThreads = 0;
	parameters.method = pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::HARRIS;

	configurationFilePath = "";
	}

HarrisDetector3D::~HarrisDetector3D()
	{

	}

void HarrisDetector3D::configure()
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


void HarrisDetector3D::process() 
	{
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputPointCloud = 
		ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Convert(inPointCloud);
	ValidateInputs(inputPointCloud);
	cv::Mat harrisPoints = ComputeHarrisPoints(inputPointCloud);
	outFeaturesSet = ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Convert(harrisPoints);
	}


cv::Mat HarrisDetector3D::ComputeHarrisPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud)
	{
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector; 
    	detector.setNonMaxSupression (parameters.nonMaxSuppression);
    	detector.setRadius (parameters.radius);
    	detector.setRadiusSearch (parameters.searchRadius);
	detector.setMethod (parameters.method);
	detector.setThreshold(parameters.detectionThreshold);
	detector.setRefine(parameters.enableRefinement);
	detector.setNumberOfThreads(parameters.numberOfThreads);
	//ISSUE: should we use this one more parameter? 
	//detector.setSearchSurface(pointCloud);
    	detector.setInputCloud(pointCloud);

    	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
    	detector.compute(*keypoints); 
	
	std::vector<unsigned> validIndicesVector;
	for(unsigned pointIndex = 0; pointIndex < keypoints->points.size(); pointIndex++)
		{
		bool validPoint = keypoints->points.at(pointIndex).x == keypoints->points.at(pointIndex).x &&
				  keypoints->points.at(pointIndex).y == keypoints->points.at(pointIndex).y &&
				  keypoints->points.at(pointIndex).z == keypoints->points.at(pointIndex).z;
		if (validPoint)
			validIndicesVector.push_back(pointIndex);
		}	
	
	cv::Mat featuresVector(validIndicesVector.size(), 3, CV_32FC1, cv::Scalar(0));
	for(unsigned indexPosition = 0; indexPosition < validIndicesVector.size(); indexPosition++)
		{
		unsigned validIndex = validIndicesVector.at(indexPosition);
		featuresVector.at<float>(indexPosition, 0) = keypoints->points.at(validIndex).x;
		featuresVector.at<float>(indexPosition, 1) = keypoints->points.at(validIndex).y;
		featuresVector.at<float>(indexPosition, 2) = keypoints->points.at(validIndex).z;
		}

	return featuresVector;
	}

HarrisDetector3D::HarrisMethod HarrisDetector3D::ConvertToMethod(std::string method)
	{
	if (method == "Harris" || method == "0")
		{
		return pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::HARRIS;
		}
	else if (method == "Noble" || method == "1")
		{
		return pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::NOBLE;
		}
	else if (method == "Lowe" || method == "2")
		{
		return pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::LOWE;
		}
	else if (method == "Tomasi" || method == "3")
		{
		return pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::TOMASI;
		}
	else if (method == "Curvature" || method == "4")
		{
		return pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::CURVATURE;
		}

	ASSERT(false, "HarrisDetector3D Configuration error, Harris Method is not one of {Harris, Noble, Lowe, Tomasi, Curvature}");
	}


void HarrisDetector3D::ValidateParameters()
	{
	ASSERT( parameters.radius > 0, "HarrisDetector3D Configuration error, radius is not positive");
	ASSERT( parameters.numberOfThreads >= 0, "HarrisDetector3D Configuration error, number of threads is negative");
	VERIFY( parameters.nonMaxSuppression || parameters.detectionThreshold < std::numeric_limits<float>::epsilon(), "Warning HarrisDetector3D: ineffective threshold when non max suppression is false");
	}

void HarrisDetector3D::ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud)
	{

	}


void HarrisDetector3D::Configure(const YAML::Node& configurationNode)
	{
	std::string nodeName = configurationNode["Name"].as<std::string>();
	if ( nodeName == "GeneralParameters")
		{
		parameters.radius = configurationNode["Radius"].as<float>();
		parameters.searchRadius = configurationNode["SearchRadius"].as<float>();
		parameters.nonMaxSuppression = configurationNode["NonMaxSuppression"].as<bool>();
		parameters.detectionThreshold = configurationNode["DetectionThreshold"].as<float>();
		parameters.enableRefinement = configurationNode["EnableRefinement"].as<bool>();
		parameters.numberOfThreads = configurationNode["NumberOfThreads"].as<int>();
		parameters.method = ConvertToMethod( configurationNode["HarrisMethod"].as<std::string>() );
		}
	//Ignore everything else
	}


}


/** @} */
