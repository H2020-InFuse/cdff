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
#include <ImageTypeToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <tinyxml2.h>
#include <XmlHelper/XmlHelper.hpp>

#include <stdlib.h>
#include <fstream>

using namespace Types;
using namespace Common;
using namespace tinyxml2;

namespace dfn_ci {


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
    	XMLDocument configuration;
    	XMLError error = configuration.LoadFile( configurationFilePath.c_str() );
	ASSERT(error == XML_SUCCESS, "HarrisDetector2D Configuration file could not be loaded");

	XMLElement* root = configuration.FirstChildElement("HarrisDetector2D");
	ASSERT(root != NULL, "Error in configuration file, no root element HarrisDetector2d");

	XMLElement* generalGroupElement = root->FirstChildElement("General");
	VERIFY(generalGroupElement != NULL, "Warning in Harris Detector 2D Configuration: no general parameters, default ones will be used");
	if (generalGroupElement != NULL)
		{
		/* XmlHelper::ExtractTYPE(A, B, C, D): 
		* it looks for name B in element A and populates variable C with the value found, if the value is not of the right type message D is written to log. */ 
		XmlHelper::ExtractInt(generalGroupElement, "ApertureSize", parameters.apertureSize, "HarrisDetector2D Configuration error, Aperture Size is not an integer");
		XmlHelper::ExtractInt(generalGroupElement, "BlockSize", parameters.blockSize, "HarrisDetector2D Configuration error, Block Size is not an integer");
		XmlHelper::ExtractFloat(generalGroupElement, "ParameterK", parameters.parameterK, "HarrisDetector2D Configuration error, Parameter K is not a float");
		XmlHelper::ExtractInt(generalGroupElement, "DetectionThreshold", parameters.detectionThreshold, "HarrisDetector2D Configuration error, Detection Threshold is not an integer");
		XmlHelper::ExtractBool(generalGroupElement, "UseGaussianBlur", parameters.useGaussianBlur, "HarrisDetector2D Configuration error, Use Gaussian Blur is not a bool");
		}

	XMLElement* gaussianBlurGroupElement = root->FirstChildElement("GaussianBlur");
	VERIFY(!parameters.useGaussianBlur || gaussianBlurGroupElement != NULL, "Warning in Harris Detector 2D Configuration: no gaussian blur parameters");
	if (parameters.useGaussianBlur && gaussianBlurGroupElement != NULL)
		{
		XmlHelper::ExtractInt(gaussianBlurGroupElement, "KernelWidth", gaussianBlurParameters.kernelWidth, "HarrisDetector2D Kernel Width is not an integer");
		XmlHelper::ExtractInt(gaussianBlurGroupElement, "KernelHeight", gaussianBlurParameters.kernelHeight, "HarrisDetector2D Kernel Height is not an integer");
		XmlHelper::ExtractFloat(gaussianBlurGroupElement, "WidthStandardDeviation", gaussianBlurParameters.widthStandardDeviation, "HarrisDetector2D Width Standard Deviation is not a float");
		XmlHelper::ExtractFloat(gaussianBlurGroupElement, "HeightStandardDeviation", gaussianBlurParameters.heightStandardDeviation, "HarrisDetector2D Height Standard Deviation is not a float");
		}

	ValidateParameters();
	}


void HarrisDetector2D::process() 
	{
	cv::Mat inputImage = ConversionCache<ImageType*, cv::Mat, ImageTypeToMatConverter>::Convert(inImage);
	ValidateInputs(inputImage);
	cv::Mat harrisImage = ComputeHarrisImage(inputImage);
	cv::Mat harrisPoints = ExtractHarrisPoints(harrisImage);
	outFeaturesSet = ConversionCache<cv::Mat, VisualPointFeatureVector2D*, MatToVisualPointFeatureVector2DConverter>::Convert(harrisPoints);
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

	cv::Mat harrisPointsList(numberOfPoints, 2, CV_16UC1, cv::Scalar(0));
	unsigned pointIndex = 0;

	for(int rowIndex = 0; rowIndex < harrisImage.rows; rowIndex++)
	for(int columnIndex = 0; columnIndex < harrisImage.cols; columnIndex++)
	if (harrisImage.at<uint8_t>(rowIndex, columnIndex) > parameters.detectionThreshold)
		{
		harrisPointsList.at<uint16_t>(pointIndex, 1) = (uint16_t)rowIndex;
		harrisPointsList.at<uint16_t>(pointIndex, 0) = (uint16_t)columnIndex;
		pointIndex++;
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


}


/** @} */
