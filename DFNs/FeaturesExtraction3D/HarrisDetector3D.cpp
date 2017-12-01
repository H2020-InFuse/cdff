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
#include <PointCloud3DToPclPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <tinyxml2.h>
#include <XmlHelper/XmlHelper.hpp>

#include <stdlib.h>
#include <fstream>


using namespace Common;
using namespace tinyxml2;
using namespace Converters;

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
	parameters.radius = 0.1;
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
    	XMLDocument configuration;
	PRINT_TO_LOG("CONGF: ", configurationFilePath);
    	XMLError error = configuration.LoadFile( configurationFilePath.c_str() );
	ASSERT(error == XML_SUCCESS, "HarrisDetector3D Configuration file could not be loaded");

	XMLElement* root = configuration.FirstChildElement("HarrisDetector3D");
	ASSERT(root != NULL, "Error in configuration file, no root element HarrisDetector3d");

	XMLElement* generalGroupElement = root->FirstChildElement("General");
	VERIFY(generalGroupElement != NULL, "Warning in Harris Detector 3D Configuration: no general parameters, default ones will be used");
	if (generalGroupElement != NULL)
		{
		/* XmlHelper::ExtractTYPE(A, B, C, D): 
		* it looks for name B in element A and populates variable C with the value found, if the value is not of the right type message D is written to log. */ 
		XmlHelper::ExtractFloat(generalGroupElement, "Radius", parameters.radius, "HarrisDetector2D Configuration error, Radius is not a float");
		XmlHelper::ExtractBool(generalGroupElement, "NonMaxSuppression", parameters.nonMaxSuppression, "HarrisDetector3D Configuration error, Non Max Suppression is not a bool");
		XmlHelper::ExtractFloat(generalGroupElement, "DetectionThreshold", parameters.detectionThreshold, "HarrisDetector2D Configuration error, Detection Threshold is not a float");
		XmlHelper::ExtractBool(generalGroupElement, "EnableRefinement", parameters.enableRefinement, "HarrisDetector3D Configuration error, Enable Refinement is not a bool");
		XmlHelper::ExtractInt(generalGroupElement, "NumberOfThreads", parameters.numberOfThreads, "HarrisDetector3D Configuration error, Number Of Threads is not an integer");

		std::string harrisMethod;
		XmlHelper::ExtractString(generalGroupElement, "HarrisMethod", harrisMethod, "HarrisDetector3D Configuration error, Harris Method is not a string");
		parameters.method = ConvertToMethod(harrisMethod);
		}

	ValidateParameters();
	}


void HarrisDetector3D::process() 
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputPointCloud = ConversionCache<PointCloud3D*, pcl::PointCloud<pcl::PointXYZ>::Ptr, PointCloud3DToPclPointCloudConverter>::Convert(inPointCloud);
	ValidateInputs(inputPointCloud);
	cv::Mat harrisPoints = ComputeHarrisPoints(inputPointCloud);
	outFeaturesSet = ConversionCache<cv::Mat, VisualPointFeatureVector3D*, MatToVisualPointFeatureVector3DConverter>::Convert(harrisPoints);
	}


cv::Mat HarrisDetector3D::ComputeHarrisPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
	{
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector; 
    	detector.setNonMaxSupression (parameters.nonMaxSuppression);
    	detector.setRadius (parameters.radius);
	detector.setMethod (parameters.method);
	detector.setThreshold(parameters.detectionThreshold);
	detector.setRefine(parameters.enableRefinement);
	detector.setNumberOfThreads(parameters.numberOfThreads);
	//ISSUE: should we use this one more parameter? 
	//detector.setSearchSurface(pointCloud);
    	detector.setInputCloud(pointCloud);

    	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
    	detector.compute(*keypoints); 
	
	cv::Mat featuresVector(keypoints->points.size(), 3, CV_32FC1, cv::Scalar(0));
	for(int pointIndex = 0; pointIndex < keypoints->points.size(); pointIndex++)
		{
		featuresVector.at<float>(pointIndex, 0) = keypoints->points.at(pointIndex).x;
		featuresVector.at<float>(pointIndex, 1) = keypoints->points.at(pointIndex).y;
		featuresVector.at<float>(pointIndex, 2) = keypoints->points.at(pointIndex).z;
		}

	return featuresVector;
	}

HarrisDetector3D::HarrisMethod HarrisDetector3D::ConvertToMethod(std::string method)
	{
	if (method == "Harris")
		{
		return pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::HARRIS;
		}
	else if (method == "Noble")
		{
		return pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::NOBLE;
		}
	else if (method == "Lowe")
		{
		return pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::LOWE;
		}
	else if (method == "Tomasi")
		{
		return pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::TOMASI;
		}
	else if (method == "Curvature")
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

void HarrisDetector3D::ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
	{

	}


}


/** @} */
