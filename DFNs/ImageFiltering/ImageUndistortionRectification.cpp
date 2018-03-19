/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageUndistortionRectification.cpp
 * @date 09/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the ImageUndistortionRectification class.
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
#include "ImageUndistortionRectification.hpp"
#include <Errors/Assert.hpp>
#include <FrameToMatConverter.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <stdlib.h>
#include <fstream>

using namespace Helpers;
using namespace Converters;
using namespace Common;

namespace dfn_ci {

using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ImageUndistortionRectification::ImageUndistortionRectification()
	{
	parametersHelper.AddParameter<float>("GeneralParameters", "ConstantBorderValue", parameters.constantBorderValue, DEFAULT_PARAMETERS.constantBorderValue);
	parametersHelper.AddParameter<std::string>("GeneralParameters", "TransformMapsFilePath", parameters.transformMapsFilePath, DEFAULT_PARAMETERS.transformMapsFilePath);
	parametersHelper.AddParameter<InterpolationMethod, InterpolationMethodHelper>("GeneralParameters", "InterpolationMethod", parameters.interpolationMethod, DEFAULT_PARAMETERS.interpolationMethod);
	parametersHelper.AddParameter<CameraConfigurationMode, CameraConfigurationModeHelper>("GeneralParameters", "CameraConfigurationMode", 
			parameters.cameraConfigurationMode, DEFAULT_PARAMETERS.cameraConfigurationMode);
	parametersHelper.AddParameter<BorderMode, BorderModeHelper>("GeneralParameters", "BorderMode", parameters.borderMode, DEFAULT_PARAMETERS.borderMode);

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

	parametersHelper.AddParameter<int>("ImageSize", "Width", parameters.imageSize.width, DEFAULT_PARAMETERS.imageSize.width);
	parametersHelper.AddParameter<int>("ImageSize", "Height", parameters.imageSize.height, DEFAULT_PARAMETERS.imageSize.height);

	for(unsigned row = 0; row < 3; row++)
		{
		for (unsigned column = 0; column < 3; column++)
			{
			std::stringstream elementStream;
			elementStream << "Element_" << row <<"_"<< column;
			parametersHelper.AddParameter<float>("RectificationMatrix", elementStream.str(), parameters.rectificationMatrix[3*row+column], DEFAULT_PARAMETERS.rectificationMatrix[3*row+column]);
			}
		}

	configurationFilePath = "";
	ConvertParametersToCvMatrices();
	}

ImageUndistortionRectification::~ImageUndistortionRectification()
	{

	}

void ImageUndistortionRectification::configure()
	{
	parametersHelper.ReadFile(configurationFilePath);
	if (parameters.cameraConfigurationMode == IN_CONFIGURATION_FILE)
		{
		ConvertParametersToCvMatrices(); //CameraMatrix, distortionCoefficients and RectificationMatrix are converted in cv::Mat format
		ComputeUndistortionRectificationMap();
		}
	else
		{
		LoadUndistortionRectificationMaps();
		}
	ValidateParameters();
	}


void ImageUndistortionRectification::process() 
	{
	cv::Mat inputImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inImage);
	ValidateInputs(inputImage);
	cv::Mat filteredImage = UndistortAndRectify(inputImage);
	outFilteredImage = ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Convert(filteredImage);
	}

ImageUndistortionRectification::CameraConfigurationModeHelper::CameraConfigurationModeHelper(const std::string& parameterName, 
		CameraConfigurationMode& boundVariable, const CameraConfigurationMode& defaultValue) :
	ParameterHelper(parameterName, boundVariable, defaultValue)
	{

	}

ImageUndistortionRectification::CameraConfigurationMode ImageUndistortionRectification::CameraConfigurationModeHelper::Convert(const std::string& configurationMode)
	{
	if (configurationMode == "Internal" || configurationMode == "0")
		{
		return IN_CONFIGURATION_FILE;
		}
	else if (configurationMode == "External" || configurationMode == "1")
		{
		return EXTERNAL_RECTIFICATION_TRANSFORM;
		}
	ASSERT(false, "ImageUndistortionRectification Error: unhandled camera configuration mode");
	return IN_CONFIGURATION_FILE;
	}

ImageUndistortionRectification::InterpolationMethodHelper::InterpolationMethodHelper(const std::string& parameterName, InterpolationMethod& boundVariable, const InterpolationMethod& defaultValue) :
	ParameterHelper(parameterName, boundVariable, defaultValue)
	{

	}

ImageUndistortionRectification::InterpolationMethod ImageUndistortionRectification::InterpolationMethodHelper::Convert(const std::string& outputFormat)
	{
	if (outputFormat == "Nearest" || outputFormat == "0")
		{
		return NEAREST;
		}
	else if (outputFormat == "Linear" || outputFormat == "1")
		{
		return LINEAR;
		}
	else if (outputFormat == "Cubic" || outputFormat == "2")
		{
		return CUBIC;
		}
	else if (outputFormat == "Lanczos" || outputFormat == "3")
		{
		return LANCZOS;
		}
	ASSERT(false, "ImageUndistortionRectification Error: unhandled interpolation method");
	return LINEAR;
	}

ImageUndistortionRectification::BorderModeHelper::BorderModeHelper(const std::string& parameterName, BorderMode& boundVariable, const BorderMode& defaultValue) :
	ParameterHelper(parameterName, boundVariable, defaultValue)
	{

	}

ImageUndistortionRectification::BorderMode ImageUndistortionRectification::BorderModeHelper::Convert(const std::string& outputFormat)
	{
	if (outputFormat == "Constant" || outputFormat == "0")
		{
		return CONSTANT;
		}
	else if (outputFormat == "Wrap" || outputFormat == "1")
		{
		return WRAP;
		}
	else if (outputFormat == "Reflect" || outputFormat == "2")
		{
		return REFLECT;
		}
	ASSERT(false, "ImageUndistortionRectification Error: unhandled border mode");
	return CONSTANT;
	}

const ImageUndistortionRectification::ImageUndistortionRectificationOptionsSet ImageUndistortionRectification::DEFAULT_PARAMETERS =
	{
	.interpolationMethod = LINEAR,
	.borderMode = CONSTANT,
	.constantBorderValue = 0,
	.transformMapsFilePath = "../../tests/ConfigurationFiles/DFNs/ImageFiltering/ImageUndistortionRectificationTransformMapsRight.yaml",
	.cameraConfigurationMode = EXTERNAL_RECTIFICATION_TRANSFORM,
	.cameraMatrix = 
		{
		.focalLengthX = 1,
		.focalLengthY = 1,
		.principlePointX = 0,
		.principlePointY = 0
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
		},
	.imageSize = 
		{
		.width = 800,
		.height = 600
		},
	.rectificationMatrix =
		{
		1, 0, 0,
		0, 1, 0,
		0, 0, 1
		}
	};


void ImageUndistortionRectification::LoadUndistortionRectificationMaps()
	{
	cv::FileStorage file(parameters.transformMapsFilePath,  cv::FileStorage::READ);
	ASSERT(file.isOpened(), "ImageUndistortionRectification configuration error: bad file path for transform Map");
	file["Map1"] >> transformMap1;
	file["Map2"] >> transformMap2;
	file.release();
	}

cv::Mat ImageUndistortionRectification::UndistortAndRectify(cv::Mat inputImage)
	{
	int borderMode;
	switch(parameters.borderMode)
		{
		case CONSTANT: borderMode = cv::BORDER_CONSTANT; break;
		case WRAP: borderMode = cv::BORDER_WRAP; break;
		case REFLECT: borderMode = cv::BORDER_REFLECT_101; break;
		default: ASSERT(false, "ImageUndistortionRectification: Unhandled border mode");
		}

	int interpolationMethod;
	switch(parameters.interpolationMethod)
		{
		case NEAREST: interpolationMethod = cv::INTER_NEAREST; break;
		case LINEAR: interpolationMethod = cv::INTER_LINEAR; break;
		case CUBIC: interpolationMethod = cv::INTER_CUBIC; break;
		case LANCZOS: interpolationMethod = cv::INTER_LANCZOS4; break;
		default: ASSERT(false, "ImageUndistortionRectification: Unhandled interpolation method");
		}

	cv::Mat filteredImage;
	cv::remap(inputImage, filteredImage, transformMap1, transformMap2, interpolationMethod, borderMode, parameters.constantBorderValue);
	
	return filteredImage;
	}


void ImageUndistortionRectification::ValidateParameters()
	{
	ASSERT(transformMap1.cols > 0 && transformMap1.rows > 0, "Image Undistortion error: transformMap1 is empty");
	ASSERT(transformMap1.size() == transformMap2.size(), "Image Undistortion error: transformMap1 size does not match transformMap2 size");

	ASSERT(parameters.cameraMatrix.focalLengthX > 0 && parameters.cameraMatrix.focalLengthY > 0, "Image Undistortion Configuration error: focal length has to be positive");
	ASSERT(!parameters.distortionParametersSet.useK4ToK6 || parameters.distortionParametersSet.useK3, "Image Undistortion Configuration error: cannot use K4,K5,K6 without K3");
	}

void ImageUndistortionRectification::ValidateInputs(cv::Mat inputImage)
	{
	ASSERT(inputImage.type() == CV_8UC3 || inputImage.type() == CV_8UC1, "Image Undistortion error: input image is not of type CV_8UC3 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "Image Undistortion error: input image is empty");
	ASSERT(inputImage.size() == transformMap1.size(), "Image Undistortion error: input image does not match transform map");
	}

void ImageUndistortionRectification::ConvertParametersToCvMatrices()
	{
	cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar(0));
	cameraMatrix.at<float>(0,0) = parameters.cameraMatrix.focalLengthX;
	cameraMatrix.at<float>(1,1) = parameters.cameraMatrix.focalLengthY;
	cameraMatrix.at<float>(0,2) = parameters.cameraMatrix.principlePointX;
	cameraMatrix.at<float>(1,2) = parameters.cameraMatrix.principlePointY;
	cameraMatrix.at<float>(2,2) = 1;

	distortionVector = cv::Mat(1, 14, CV_32FC1, cv::Scalar(0));
	distortionVector.at<float>(0,0) = parameters.distortionParametersSet.k1;
	distortionVector.at<float>(0,1) = parameters.distortionParametersSet.k2;
	distortionVector.at<float>(0,2) = parameters.distortionParametersSet.p1;
	distortionVector.at<float>(0,3) = parameters.distortionParametersSet.p2;
	if (parameters.distortionParametersSet.useK3)
		{
		distortionVector.at<float>(0,4) = parameters.distortionParametersSet.k3;
		}
	if (parameters.distortionParametersSet.useK4ToK6)
		{
		distortionVector.at<float>(0,5) = parameters.distortionParametersSet.k4;
		distortionVector.at<float>(0,6) = parameters.distortionParametersSet.k5;
		distortionVector.at<float>(0,7) = parameters.distortionParametersSet.k6;
		}

	rectificationMatrix = cv::Mat(3, 3, CV_32FC1);
	for(unsigned row = 0; row < 3; row++)
		{
		for(unsigned column = 0; column < 3; column++)
			{
			rectificationMatrix.at<float>(row, column) = parameters.rectificationMatrix[3*row+column];
			}
		}
	}

void ImageUndistortionRectification::ComputeUndistortionRectificationMap()
	{
	cv::Size imageSize(parameters.imageSize.width, parameters.imageSize.height);
	cv::Mat optimalNewCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distortionVector, imageSize, 1);

	cv::initUndistortRectifyMap
		(
		cameraMatrix,
		distortionVector,
		rectificationMatrix,
		optimalNewCameraMatrix,
		imageSize,
		CV_32FC1,
		transformMap1, 
		transformMap2
		);
	}

}


/** @} */
