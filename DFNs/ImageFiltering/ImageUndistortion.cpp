/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "ImageUndistortion.hpp"

#include <Errors/Assert.hpp>
#include <Macros/YamlcppMacros.hpp>

#include <stdlib.h>
#include <fstream>

using namespace Converters;
using namespace FrameWrapper;

namespace CDFF
{
namespace DFN
{
namespace ImageFiltering
{

ImageUndistortion::ImageUndistortion()
{
	parametersHelper.AddParameter<float>("CameraMatrix", "FocalLengthX", parameters.cameraMatrix.focalLengthX, DEFAULT_PARAMETERS.cameraMatrix.focalLengthX);
	parametersHelper.AddParameter<float>("CameraMatrix", "FocalLengthY", parameters.cameraMatrix.focalLengthY, DEFAULT_PARAMETERS.cameraMatrix.focalLengthY);
	parametersHelper.AddParameter<float>("CameraMatrix", "PrinciplePointX", parameters.cameraMatrix.principlePointX, DEFAULT_PARAMETERS.cameraMatrix.principlePointX);
	parametersHelper.AddParameter<float>("CameraMatrix", "PrinciplePointY", parameters.cameraMatrix.principlePointY, DEFAULT_PARAMETERS.cameraMatrix.principlePointY);

	parametersHelper.AddParameter<float>("Distortion", "K1", parameters.distortionParametersSet.k1, DEFAULT_PARAMETERS.distortionParametersSet.k1);
	parametersHelper.AddParameter<float>("Distortion", "K2", parameters.distortionParametersSet.k2, DEFAULT_PARAMETERS.distortionParametersSet.k2);
	parametersHelper.AddParameter<float>("Distortion", "K3", parameters.distortionParametersSet.k3, DEFAULT_PARAMETERS.distortionParametersSet.k3);
	parametersHelper.AddParameter<float>("Distortion", "K4", parameters.distortionParametersSet.k4, DEFAULT_PARAMETERS.distortionParametersSet.k4);
	parametersHelper.AddParameter<float>("Distortion", "K5", parameters.distortionParametersSet.k5, DEFAULT_PARAMETERS.distortionParametersSet.k5);
	parametersHelper.AddParameter<float>("Distortion", "K6", parameters.distortionParametersSet.k6, DEFAULT_PARAMETERS.distortionParametersSet.k6);
	parametersHelper.AddParameter<float>("Distortion", "P1", parameters.distortionParametersSet.p1, DEFAULT_PARAMETERS.distortionParametersSet.p1);
	parametersHelper.AddParameter<float>("Distortion", "P2", parameters.distortionParametersSet.p2, DEFAULT_PARAMETERS.distortionParametersSet.p2);

	parametersHelper.AddParameter<bool>("Distortion", "UseK3", parameters.distortionParametersSet.useK3, DEFAULT_PARAMETERS.distortionParametersSet.useK3);
	parametersHelper.AddParameter<bool>("Distortion", "UseK4ToK6", parameters.distortionParametersSet.useK4ToK6, DEFAULT_PARAMETERS.distortionParametersSet.useK4ToK6);

	configurationFilePath = "";
}

ImageUndistortion::~ImageUndistortion()
{
}

void ImageUndistortion::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void ImageUndistortion::process()
{
	// Read data from input port
	cv::Mat inputImage = frameToMat.Convert(&inImage);

	// Process data
	ValidateInputs(inputImage);
	cv::Mat undistortedImage = Undistort(inputImage);

	// Write data to output port
	FrameConstPtr tmp = matToFrame.Convert(undistortedImage);
	Copy(*tmp, outImage);
	delete(tmp);
}

const ImageUndistortion::ImageUndistortionOptionsSet ImageUndistortion::DEFAULT_PARAMETERS =
{
	.cameraMatrix =
	{
		.focalLengthX = 1,
		.focalLengthY = 1,
		.principlePointX = 0,
		.principlePointY = 0,
	},
	.distortionParametersSet =
	{
		.k1 = 0,
		.k2 = 0,
		.k3 = 0,
		.k4 = 0,
		.k5 = 0,
		.k6 = 0,
		.p1 = 0,
		.p2 = 0,
		.useK3 = false,
		.useK4ToK6 = false
	}
};

cv::Mat ImageUndistortion::Undistort(cv::Mat inputImage)
{
	cv::Mat cameraMatrix(3, 3, CV_32FC1, cv::Scalar(0));
	cameraMatrix.at<float>(0,0) = parameters.cameraMatrix.focalLengthX;
	cameraMatrix.at<float>(1,1) = parameters.cameraMatrix.focalLengthY;
	cameraMatrix.at<float>(0,2) = parameters.cameraMatrix.principlePointX;
	cameraMatrix.at<float>(1,2) = parameters.cameraMatrix.principlePointY;
	cameraMatrix.at<float>(2,2) = 1;

	cv::Mat distortionCoefficients;
	if (parameters.distortionParametersSet.useK4ToK6)
	{
		distortionCoefficients = cv::Mat(1, 8, CV_32FC1);
		distortionCoefficients.at<float>(0,4) = parameters.distortionParametersSet.k3;
		distortionCoefficients.at<float>(0,5) = parameters.distortionParametersSet.k4;
		distortionCoefficients.at<float>(0,6) = parameters.distortionParametersSet.k5;
		distortionCoefficients.at<float>(0,7) = parameters.distortionParametersSet.k6;
	}
	else if (parameters.distortionParametersSet.useK3)
	{
		distortionCoefficients = cv::Mat(1, 5, CV_32FC1);
		distortionCoefficients.at<float>(0,4) = parameters.distortionParametersSet.k3;
	}
	else
	{
		distortionCoefficients = cv::Mat(1, 4, CV_32FC1);
	}
	distortionCoefficients.at<float>(0,0) = parameters.distortionParametersSet.k1;
	distortionCoefficients.at<float>(0,1) = parameters.distortionParametersSet.k2;
	distortionCoefficients.at<float>(0,2) = parameters.distortionParametersSet.p1;
	distortionCoefficients.at<float>(0,3) = parameters.distortionParametersSet.p2;

	cv::Mat undistortedImage;
	cv::undistort(inputImage, undistortedImage, cameraMatrix, distortionCoefficients);

	return undistortedImage;
}

void ImageUndistortion::ValidateParameters()
{
	ASSERT(parameters.cameraMatrix.focalLengthX > 0 && parameters.cameraMatrix.focalLengthY > 0, "Image Undistortion Configuration error: focal length has to be positive");
	ASSERT(!parameters.distortionParametersSet.useK4ToK6 || parameters.distortionParametersSet.useK3, "Image Undistortion Configuration error: cannot use K4,K5,K6 without K3");
}

void ImageUndistortion::ValidateInputs(cv::Mat inputImage)
{
	ASSERT(inputImage.type() == CV_8UC3 || inputImage.type() == CV_8UC1, "Image Undistortion error: input image is not of type CV_8UC3 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "Image Undistortion error: input image is empty");
}

}
}
}

/** @} */
