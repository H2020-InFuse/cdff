/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "HarrisDetector2D.hpp"
#include <FrameToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <fstream>

using namespace Helpers;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace FrameWrapper;

namespace dfn_ci
{

HarrisDetector2D::HarrisDetector2D()
{
	parametersHelper.AddParameter<int>("GeneralParameters", "ApertureSize", parameters.generalParameters.apertureSize, DEFAULT_PARAMETERS.generalParameters.apertureSize);
	parametersHelper.AddParameter<int>("GeneralParameters", "BlockSize", parameters.generalParameters.blockSize, DEFAULT_PARAMETERS.generalParameters.blockSize);
	parametersHelper.AddParameter<float>("GeneralParameters", "ParameterK", parameters.generalParameters.parameterK, DEFAULT_PARAMETERS.generalParameters.parameterK);
	parametersHelper.AddParameter<int>("GeneralParameters", "DetectionThreshold", parameters.generalParameters.detectionThreshold, DEFAULT_PARAMETERS.generalParameters.detectionThreshold);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseGaussianBlur", parameters.generalParameters.useGaussianBlur, DEFAULT_PARAMETERS.generalParameters.useGaussianBlur);

	parametersHelper.AddParameter<int>("GaussianBlur", "KernelWidth", parameters.gaussianBlurParameters.kernelWidth, DEFAULT_PARAMETERS.gaussianBlurParameters.kernelWidth);
	parametersHelper.AddParameter<int>("GaussianBlur", "KernelHeight", parameters.gaussianBlurParameters.kernelHeight, DEFAULT_PARAMETERS.gaussianBlurParameters.kernelHeight);
	parametersHelper.AddParameter<float>("GaussianBlur", "WidthStandardDeviation", parameters.gaussianBlurParameters.widthStandardDeviation, DEFAULT_PARAMETERS.gaussianBlurParameters.widthStandardDeviation);
	parametersHelper.AddParameter<float>("GaussianBlur", "HeightStandardDeviation", parameters.gaussianBlurParameters.heightStandardDeviation, DEFAULT_PARAMETERS.gaussianBlurParameters.heightStandardDeviation);

	configurationFilePath = "";
}

HarrisDetector2D::~HarrisDetector2D()
{
}

void HarrisDetector2D::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void HarrisDetector2D::process()
{
	// Read data from input port
	cv::Mat inputImage = frameToMat.Convert(&inFrame);

	// Process data
	ValidateInputs(inputImage);
	cv::Mat harrisImage = ComputeHarrisImage(inputImage);
	cv::Mat harrisPoints = ExtractHarrisPoints(harrisImage);

	// Write data to output port
	VisualPointFeatureVector2DConstPtr tmp = matToVisualPointFeatureVector2D.Convert(harrisPoints);
	Copy(*tmp, outFeatures);
	delete(tmp);
}

const HarrisDetector2D::HarrisOptionsSet HarrisDetector2D::DEFAULT_PARAMETERS =
{
	.generalParameters =
	{
		.apertureSize = 3,
		.blockSize = 3,
		.parameterK = 0.5,
		.detectionThreshold = 45,
		.useGaussianBlur = false
	},

	.gaussianBlurParameters =
	{
		.kernelWidth = 0,
		.kernelHeight = 0,
		.widthStandardDeviation = 0,
		.heightStandardDeviation = 0
	}
};

cv::Mat HarrisDetector2D::ComputeHarrisImage(cv::Mat inputImage)
{
	cv::Mat grayImage;
	cv::cvtColor(inputImage, grayImage, CV_BGR2GRAY);

	cv::Mat blurredImage;
	if (parameters.generalParameters.useGaussianBlur)
	{
		cv::GaussianBlur(
			grayImage,
			blurredImage,
			cv::Size(parameters.gaussianBlurParameters.kernelWidth, parameters.gaussianBlurParameters.kernelHeight),
			parameters.gaussianBlurParameters.widthStandardDeviation,
			parameters.gaussianBlurParameters.heightStandardDeviation);
	}
	else
	{
		blurredImage = grayImage;
	}

	cv::Mat harrisMatrix = cv::Mat(inputImage.size(), CV_32FC1);
	cv::cornerHarris(blurredImage, harrisMatrix, parameters.generalParameters.blockSize, parameters.generalParameters.apertureSize, parameters.generalParameters.parameterK, cv::BORDER_DEFAULT);

	cv::Mat normalizedHarrisMatrix;
	cv::normalize(harrisMatrix, normalizedHarrisMatrix, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

	cv::Mat harrisImage;
	cv::convertScaleAbs(normalizedHarrisMatrix, harrisImage);

	return harrisImage;
}

cv::Mat HarrisDetector2D::ExtractHarrisPoints(cv::Mat harrisImage)
{
	int numberOfPoints = cv::countNonZero(harrisImage > parameters.generalParameters.detectionThreshold);
	int numberOfAllowedPoints = (numberOfPoints < VisualPointFeatureVector2DWrapper::MAX_FEATURE_2D_POINTS) ? numberOfPoints : VisualPointFeatureVector2DWrapper::MAX_FEATURE_2D_POINTS;

	cv::Mat harrisPointsList(numberOfAllowedPoints, 3, CV_32FC1, cv::Scalar(0));
	unsigned point = 0;
	bool extractionComplete = false;

	for (int row = 0; row < harrisImage.rows && !extractionComplete; row++)
	{
		for (int column = 0; column < harrisImage.cols && !extractionComplete; column++)
		{
			if (harrisImage.at<uint8_t>(row, column) > parameters.generalParameters.detectionThreshold)
			{
				harrisPointsList.at<float>(point, 1) = (float)row;
				harrisPointsList.at<float>(point, 0) = (float)column;
				harrisPointsList.at<float>(point, 2) = parameters.generalParameters.blockSize;
				point++;
				extractionComplete = (point == numberOfAllowedPoints);
			}
		}
	}

	return harrisPointsList;
}

void HarrisDetector2D::ValidateParameters()
{
	ASSERT(parameters.generalParameters.apertureSize == 3 || parameters.generalParameters.apertureSize == 5 || parameters.generalParameters.apertureSize == 7,
		"Harris Detector Configuration error: Aperture size should be either 3, 5, or 7");

	if (parameters.generalParameters.useGaussianBlur)
	{
		bool useKernel = parameters.gaussianBlurParameters.kernelWidth > 0 || parameters.gaussianBlurParameters.kernelHeight > 0;
		bool useDeviation =
			parameters.gaussianBlurParameters.widthStandardDeviation > std::numeric_limits<float>::epsilon() ||
			parameters.gaussianBlurParameters.heightStandardDeviation > std::numeric_limits<float>::epsilon();

		ASSERT(useKernel || useDeviation, "Harris Detector Configuration error: no Gaussian Blur mode have been configured");
		if (useKernel)
		{
			ASSERT(parameters.gaussianBlurParameters.kernelWidth%2 == 1, "Harris Detector Configuration error: Gaussian Blur Kernel Width should be odd");
			ASSERT(parameters.gaussianBlurParameters.kernelHeight%2 == 1, "Harris Detector Configuration error: Gaussian Blur Kernel Height should be odd");
			VERIFY(!useDeviation, "Warning in Harris Detector Configuration: gaussian blur is configured both in kernel and deviation mode. Kernel mode will be used");
		}
	}
}

void HarrisDetector2D::ValidateInputs(cv::Mat inputImage)
{
	ASSERT(inputImage.type() == CV_8UC3 || inputImage.type() == CV_8UC1, "HarrisDetector2D error: input image is not of type CV_8UC3 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "HarrisDetector2D error: input image is empty");
}

}

/** @} */
