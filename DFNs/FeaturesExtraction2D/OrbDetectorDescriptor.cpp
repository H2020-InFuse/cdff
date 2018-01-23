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
	parameters.sizeOfBrightnessTestSet = DEFAULT_PARAMETERS.sizeOfBrightnessTestSet;

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
	.edgeThreshold = 31,
	.fastThreshold = 20,
	.firstLevel = 0,
	.maxFeaturesNumber = 500,
	.levelsNumber = 8,
	.patchSize = 31,
	.scaleFactor = 1.2,
	.scoreType = 0,
	.sizeOfBrightnessTestSet = 2
	};


cv::Mat OrbDetectorDescriptor::ComputeOrbFeatures(cv::Mat inputImage)
	{
	cv::Ptr<cv::ORB> orb = cv::ORB::create();
	orb->setEdgeThreshold(parameters.edgeThreshold);
	orb->setFastThreshold(parameters.fastThreshold);
	orb->setFirstLevel(parameters.firstLevel);
	orb->setMaxFeatures(parameters.maxFeaturesNumber);
	orb->setNLevels(parameters.levelsNumber);
	orb->setPatchSize(parameters.patchSize);
	orb->setScaleFactor(parameters.scaleFactor);
	orb->setScoreType(parameters.scoreType);
	orb->setWTA_K(parameters.sizeOfBrightnessTestSet);

	std::vector<cv::KeyPoint> keypointsVector;
	cv::Mat descriptorsMatrix;
	cv::Mat mask = cv::Mat();
	orb->detectAndCompute(inputImage, mask, keypointsVector, descriptorsMatrix);

	if (keypointsVector.size() == 0)
		{
		return cv::Mat();
		}

	cv::Mat orbFeaturesMatrix(keypointsVector.size(), descriptorsMatrix.cols + 2, CV_32FC1);
	cv::Mat descriptorSubmatrix = orbFeaturesMatrix(cv::Rect(2, 0, descriptorsMatrix.cols, keypointsVector.size() ) );
	descriptorsMatrix.convertTo(descriptorSubmatrix, CV_32FC1);
	for(unsigned pointIndex = 0; pointIndex < keypointsVector.size(); pointIndex++)
		{
		orbFeaturesMatrix.at<float>(pointIndex, 0) = keypointsVector.at(pointIndex).pt.x;
		orbFeaturesMatrix.at<float>(pointIndex, 1) = keypointsVector.at(pointIndex).pt.y;
		}
	
	return orbFeaturesMatrix;
	}


void OrbDetectorDescriptor::ValidateParameters()
	{
	ASSERT(parameters.edgeThreshold > 0, "Orb Detector Descriptor Configuration Error: edge threshold should be strictly positive");
	ASSERT(parameters.fastThreshold > 0, "Orb Detector Descriptor Configuration Error: fast threshold should be strictly positive");
	ASSERT(parameters.firstLevel == 0, "Orb Detector Descriptor Configuration Error: first level should be 0");
	ASSERT(parameters.maxFeaturesNumber > 0, "Orb Detector Descriptor Configuration Error: max features number should be strictly positive");
	ASSERT(parameters.levelsNumber > 0, "Orb Detector Descriptor Configuration Error: pyramid levels number should be strictly positive");
	ASSERT(parameters.patchSize > 0, "Orb Detector Descriptor Configuration Error: patch size should be strictly positive");
	ASSERT(parameters.scaleFactor > 1, "Orb Detector Descriptor Configuration Error: scale factor should be greater than 1");
	ASSERT(parameters.scoreType == cv::ORB::HARRIS_SCORE || parameters.scoreType == cv::ORB::FAST_SCORE, "Orb Detector Descriptor Configuration Error: scoreType should be HarrisScore or FastScore");
	ASSERT(parameters.sizeOfBrightnessTestSet >= 2 && parameters.sizeOfBrightnessTestSet <= 4, "Orb Detector Descriptor Configuration Error: size of brightness test set should be 2, 3, or 4");
	}

void OrbDetectorDescriptor::ValidateInputs(cv::Mat inputImage)
	{
	ASSERT(inputImage.type() == CV_8UC3 || inputImage.type() == CV_8UC1, "OrbDetectorDescriptor error: input image is not of type CV_8UC3 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "OrbDetectorDescriptor error: input image is empty");
	}

int OrbDetectorDescriptor::ConvertToScoreType(std::string scoreType)
	{
	if (scoreType == "HarrisScore" || scoreType == "0")
		{
		return cv::ORB::HARRIS_SCORE;
		}
	if (scoreType == "FastScore" || scoreType == "1")
		{
		return cv::ORB::FAST_SCORE;
		}
	ASSERT(false, "Orb Detector Descriptor Configuration Error: Score type should be either HarrisScore or FastScore (1, or 2)");
	}


void OrbDetectorDescriptor::Configure(const YAML::Node& configurationNode)
	{
	std::string nodeName = configurationNode["Name"].as<std::string>();
	if ( nodeName == "GeneralParameters")
		{
		parameters.edgeThreshold = configurationNode["EdgeThreshold"].as<int>(); 
		parameters.fastThreshold = configurationNode["FastThreshold"].as<int>(); 
		parameters.firstLevel = configurationNode["FirstLevel"].as<int>(); 
		parameters.maxFeaturesNumber = configurationNode["MaxFeaturesNumber"].as<int>(); 
		parameters.levelsNumber = configurationNode["LevelsNumber"].as<int>(); 
		parameters.patchSize = configurationNode["PatchSize"].as<int>(); 
		parameters.scaleFactor = configurationNode["ScaleFactor"].as<double>(); 
		parameters.scoreType = ConvertToScoreType( configurationNode["ScoreType"].as<std::string>() ); 
		parameters.sizeOfBrightnessTestSet = configurationNode["SizeOfBrightnessTestSet"].as<int>(); 
		}
	//Ignore everything else
	}

}


/** @} */
