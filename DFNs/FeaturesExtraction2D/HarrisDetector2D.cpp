/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisDetector2D.cpp
 * @date 17/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the Harris Detector 2D class.
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
#include "HarrisDetector2D.hpp"
#include <Errors/Assert.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>

#include <stdlib.h>
#include <fstream>

using namespace Converters;
using namespace Common;

namespace dfn_ci {

using namespace CppTypes;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
HarrisDetector2D::HarrisDetector2D()
	{
	parameters.apertureSize = 3;
	parameters.blockSize = 3;
	parameters.parameterK = 0.5;
	parameters.detectionThreshold = 45;
	parameters.useGaussianBlur = false;

	gaussianBlurParameters.kernelWidth = 0;
	gaussianBlurParameters.kernelHeight = 0;
	gaussianBlurParameters.widthStandardDeviation = 0;
	gaussianBlurParameters.heightStandardDeviation = 0;

	configurationFilePath = "";
	}

HarrisDetector2D::~HarrisDetector2D()
	{

	}

void HarrisDetector2D::configure()
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


void HarrisDetector2D::process() 
	{
	cv::Mat inputImage = ConversionCache<Frame::ConstPtr, cv::Mat, FrameToMatConverter>::Convert(inImage);
	ValidateInputs(inputImage);
	cv::Mat harrisImage = ComputeHarrisImage(inputImage);
	cv::Mat harrisPoints = ExtractHarrisPoints(harrisImage);
	outFeaturesSet = ConversionCache<cv::Mat, VisualPointFeatureVector2D::ConstPtr, MatToVisualPointFeatureVector2DConverter>::Convert(harrisPoints);
	}


cv::Mat HarrisDetector2D::ComputeHarrisImage(cv::Mat inputImage)
	{
	cv::Mat grayImage;
	cv::cvtColor(inputImage, grayImage, CV_BGR2GRAY);

	cv::Mat blurredImage;
	if(parameters.useGaussianBlur)
		cv::GaussianBlur
			(
			grayImage, 
			blurredImage, 
			cv::Size(gaussianBlurParameters.kernelWidth, gaussianBlurParameters.kernelHeight), 
			gaussianBlurParameters.widthStandardDeviation, 
			gaussianBlurParameters.heightStandardDeviation
			);
	else
		blurredImage = grayImage;
 
	cv::Mat harrisMatrix = cv::Mat(inputImage.size(), CV_32FC1);
	cv::cornerHarris(blurredImage, harrisMatrix, parameters.blockSize, parameters.apertureSize, parameters.parameterK, cv::BORDER_DEFAULT );

	cv::Mat normalizedHarrisMatrix;
 	cv::normalize( harrisMatrix, normalizedHarrisMatrix, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );

	cv::Mat harrisImage;
  	cv::convertScaleAbs(normalizedHarrisMatrix, harrisImage);
	
	return harrisImage;
	}


cv::Mat HarrisDetector2D::ExtractHarrisPoints(cv::Mat harrisImage)
	{
	int numberOfPoints = cv::countNonZero(harrisImage > parameters.detectionThreshold);

	cv::Mat harrisPointsList(numberOfPoints, 2, CV_32FC1, cv::Scalar(0));
	unsigned pointIndex = 0;

	for(int rowIndex = 0; rowIndex < harrisImage.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < harrisImage.cols; columnIndex++)
			{
			if (harrisImage.at<uint8_t>(rowIndex, columnIndex) > parameters.detectionThreshold)
				{
				harrisPointsList.at<float>(pointIndex, 1) = (float)rowIndex;
				harrisPointsList.at<float>(pointIndex, 0) = (float)columnIndex;
				pointIndex++;
				}
			}
		}		

	return harrisPointsList;
	}


void HarrisDetector2D::ValidateParameters()
	{
	ASSERT(parameters.apertureSize == 3 || parameters.apertureSize == 5 || parameters.apertureSize == 7, "Harris Detector Configuration error: Aperture size should be either 3, 5, or 7");
	
	if (parameters.useGaussianBlur)
		{
		bool useKernel = gaussianBlurParameters.kernelWidth > 0 || gaussianBlurParameters.kernelHeight > 0;
		bool useDeviation = gaussianBlurParameters.widthStandardDeviation > std::numeric_limits<float>::epsilon() || gaussianBlurParameters.heightStandardDeviation > std::numeric_limits<float>::epsilon();
		ASSERT(useKernel || useDeviation, "Harris Detector Configuration error: no Gaussian Blur mode have been configured");
		if (useKernel)
			{
			ASSERT(gaussianBlurParameters.kernelWidth%2 == 1, "Harris Detector Configuration error: Gaussian Blur Kernel Width should be odd"); 
			ASSERT(gaussianBlurParameters.kernelHeight%2 == 1, "Harris Detector Configuration error: Gaussian Blur Kernel Height should be odd"); 
			VERIFY(!useDeviation, "Warning in Harris Detector Configuration: gaussian blur is configured both in kernel and deviation mode. Kernel mode will be used");
			}
		}
	}

void HarrisDetector2D::ValidateInputs(cv::Mat inputImage)
	{

	}


void HarrisDetector2D::Configure(const YAML::Node& configurationNode)
	{
	std::string nodeName = configurationNode["Name"].as<std::string>();
	if ( nodeName == "GeneralParameters")
		{
		parameters.apertureSize = configurationNode["ApertureSize"].as<int>();
		parameters.blockSize = configurationNode["BlockSize"].as<int>();
		parameters.parameterK = configurationNode["ParameterK"].as<float>();
		parameters.detectionThreshold = configurationNode["DetectionThreshold"].as<int>();
		parameters.useGaussianBlur = configurationNode["UseGaussianBlur"].as<bool>();
		}
	else if (nodeName == "GaussianBlur")
		{
		gaussianBlurParameters.kernelWidth = configurationNode["KernelWidth"].as<int>();
		gaussianBlurParameters.kernelHeight = configurationNode["KernelHeight"].as<int>();
		gaussianBlurParameters.widthStandardDeviation = configurationNode["WidthStandardDeviation"].as<float>();
		gaussianBlurParameters.heightStandardDeviation = configurationNode["HeightStandardDeviation"].as<float>();
		}
	//Ignore everything else
	}

}


/** @} */
