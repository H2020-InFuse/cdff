/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OrbDetectorDescriptor.cpp
 * @date 23/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the ORB (FAST Detector + BRIEF descriptor) class.
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
#include "OrbDetectorDescriptor.hpp"
#include <Errors/Assert.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>

#include <stdlib.h>
#include <fstream>

using namespace Converters;
using namespace Common;

namespace dfn_ci {

using namespace VisualPointFeatureVector2DWrapper;
using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
OrbDetectorDescriptor::OrbDetectorDescriptor()
	{
	parameters.edgeThreshold = DEFAULT_PARAMETERS.edgeThreshold;
	parameters.fastThreshold = DEFAULT_PARAMETERS.fastThreshold;
	parameters.firstLevel = DEFAULT_PARAMETERS.firstLevel;
	parameters.maxFeaturesNumber = DEFAULT_PARAMETERS.maxFeaturesNumber;
	parameters.levelsNumber = DEFAULT_PARAMETERS.levelsNumber;
	parameters.patchSize = DEFAULT_PARAMETERS.patchSize;
	parameters.scaleFactor = DEFAULT_PARAMETERS.scaleFactor;
	parameters.scoreType = DEFAULT_PARAMETERS.scoreType;
	parameters.wta_k = DEFAULT_PARAMETERS.wta_k;

	configurationFilePath = "";
	}

OrbDetectorDescriptor::~OrbDetectorDescriptor()
	{

	}

void OrbDetectorDescriptor::configure()
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
	catch(YAML::RepresentationException& e)
		{
		ASSERT(false, e.what() );
		}
	}


void OrbDetectorDescriptor::process() 
	{
	cv::Mat inputImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inImage);
	ValidateInputs(inputImage);
	cv::Mat orbFeatures = ComputeOrbFeatures(inputImage);
	outFeaturesSet = ConversionCache<cv::Mat, VisualPointFeatureVector2DConstPtr, MatToVisualPointFeatureVector2DConverter>::Convert(orbFeatures);
	}

const OrbDetectorDescriptor::OrbOptionsSet OrbDetectorDescriptor::DEFAULT_PARAMETERS = 
	{
	.edgeThreshold = 0,
	.fastThreshold = 0,
	.firstLevel = 0,
	.maxFeaturesNumber = 0,
	.levelsNumber = 0,
	.patchSize = 0,
	.scaleFactor = 0.00,
	.scoreType = 0,
	.wta_k = 0
	};


cv::Mat OrbDetectorDescriptor::ComputeOrbFeatures(cv::Mat inputImage)
	{
	cv::ORB* orb = cv::ORB::create();
	orb->setEdgeThreshold(parameters.edgeThreshold);
	orb->setFastThreshold(parameters.fastThreshold);
	orb->setFirstLevel(parameters.firstLevel);
	orb->setMaxFeatures(parameters.maxFeaturesNumber);
	orb->setNLevels(parameters.levelsNumber);
	orb->setPatchSize(parameters.patchSize);
	orb->setScaleFactor(parameters.scaleFactor);
	orb->setScoreType(parameters.scoreType);
	orb->setWTA_K(parameters.wta_k);

	std::vector<cv::KeyPoint> keypointsVector;
	cv::Mat descriptorsMatrix;
	cv::Mat mask = cv::Mat();
	orb->detectAndCompute(inputImage, mask, keypointsVector, descriptorsMatrix);
	delete(orb);	

	cv::Mat orbFeaturesMatrix(keypointsVector.size(), descriptorsMatrix.cols + 2, CV_32FC1);
	cv::Mat descriptorSubmatrix = orbFeaturesMatrix(cv::Rect(0, 2, keypointsVector.size(), descriptorsMatrix.cols));
	descriptorsMatrix.copyTo(descriptorSubmatrix);
	for(int pointIndex = 0; pointIndex < keypointsVector.size(); pointIndex++)
		{
		orbFeaturesMatrix.at<float>(pointIndex, 0) = keypointsVector.at(pointIndex).pt.x;
		orbFeaturesMatrix.at<float>(pointIndex, 1) = keypointsVector.at(pointIndex).pt.y;
		}
	
	
	return orbFeaturesMatrix;
	}


void OrbDetectorDescriptor::ValidateParameters()
	{
	ASSERT(parameters.maxFeaturesNumber > 0, "Orb Detector Descriptor Configuration Error: max features number should be strictly positive");
	}

void OrbDetectorDescriptor::ValidateInputs(cv::Mat inputImage)
	{
	ASSERT(inputImage.type() == CV_8UC3 || inputImage.type() == CV_8UC1, "HarrisDetector2D error: input image is not of type CV_8UC3 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "HarrisDetector2D error: input image is empty");
	}


void OrbDetectorDescriptor::Configure(const YAML::Node& configurationNode)
	{
	std::string nodeName = configurationNode["Name"].as<std::string>();
	if ( nodeName == "GeneralParameters")
		{
		parameters.edgeThreshold = configurationNode["EdgeThreshold"].as<int>();
		parameters.fastThreshold = configurationNode["FastThreshold"].as<int>();
		parameters.firstLevel = configurationNode["FirstLevel"].as<float>();
		parameters.maxFeaturesNumber = configurationNode["MaxFeaturesNumber"].as<int>();
		parameters.levelsNumber = configurationNode["LevelsNumber"].as<bool>();
		parameters.patchSize = configurationNode["PatchSize"].as<int>();
		parameters.scaleFactor = configurationNode["ScaleFactor"].as<bool>();
		parameters.scoreType = configurationNode["ScoreType"].as<int>();
		parameters.wta_k = configurationNode["Wta_k"].as<bool>();
		}
	//Ignore everything else
	}

}


/** @} */
