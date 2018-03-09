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
	parametersHelper.AddParameter<BorderMode, BorderModeHelper>("GeneralParameters", "BorderMode", parameters.borderMode, DEFAULT_PARAMETERS.borderMode);

	configurationFilePath = "";
	}

ImageUndistortionRectification::~ImageUndistortionRectification()
	{

	}

void ImageUndistortionRectification::configure()
	{
	parametersHelper.ReadFile(configurationFilePath);
	LoadTransformMaps();
	ValidateParameters();
	}


void ImageUndistortionRectification::process() 
	{
	cv::Mat inputImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inImage);
	ValidateInputs(inputImage);
	cv::Mat filteredImage = UndistortAndRectify(inputImage);
	outFilteredImage = ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Convert(filteredImage);
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
	.transformMapsFilePath = "../../tests/ConfigurationFiles/DFNs/ImageFiltering/ImageUndistortionRectificationTransformMapsRight.yaml"
	};


void ImageUndistortionRectification::LoadTransformMaps()
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
	}

void ImageUndistortionRectification::ValidateInputs(cv::Mat inputImage)
	{
	ASSERT(inputImage.type() == CV_8UC3 || inputImage.type() == CV_8UC1, "Image Undistortion error: input image is not of type CV_8UC3 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "Image Undistortion error: input image is empty");
	ASSERT(inputImage.size() == transformMap1.size(), "Image Undistortion error: input image does not match transform map");
	}

}


/** @} */
