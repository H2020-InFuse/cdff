/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SobelScharr.cpp
 * @date 12/04/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 * 
 * Sobel (Scharroperator) Implementation .
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
#include "SobelScharr.hpp"
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
SobelScharr::SobelScharr()
	{
	parametersHelper.AddParameter<float>("GeneralParameters", "ConstantBorderValue", parameters.constantBorderValue, DEFAULT_PARAMETERS.constantBorderValue);
	parametersHelper.AddParameter<BorderMode, BorderModeHelper>("GeneralParameters", "BorderMode", parameters.borderMode, DEFAULT_PARAMETERS.borderMode);
	parametersHelper.AddParameter<DepthMode, DepthModeHelper>("GeneralParameters", "DepthMode", parameters.depthMode, DEFAULT_PARAMETERS.depthMode);
	parametersHelper.AddParameter<float>("SobelParameters", "Scale", parameters.sobelParameters.scale, DEFAULT_PARAMETERS.sobelParameters.scale);
	parametersHelper.AddParameter<float>("SobelParameters", "Delta", parameters.sobelParameters.delta, DEFAULT_PARAMETERS.sobelParameters.delta);
	
	configurationFilePath = "";
	}
SobelScharr::~SobelScharr()
{

}

void SobelScharr::configure()
	{

	 //TODO: debug ReadFile(configurationFilePath)!
	//parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
	}


void SobelScharr::process() 
	{
	cv::Mat inputImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inImage);
	ValidateInputs(inputImage);
	
	cv::Mat  scharrGradx= sobelx(inputImage);
	outSobelGradx = ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Convert(scharrGradx);

	cv::Mat  scharrGrady= sobely(inputImage);
	outSobelGrady = ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Convert(scharrGrady);
	}


SobelScharr::BorderModeHelper::BorderModeHelper(const std::string& parameterName, BorderMode& boundVariable, const BorderMode& defaultValue) :
	ParameterHelper(parameterName, boundVariable, defaultValue)
	{

	}

SobelScharr::BorderMode SobelScharr::BorderModeHelper::Convert(const std::string& outputFormat)
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
	else if (outputFormat == "border_default" || outputFormat == "3")
		{
		return BORDER_DEFAULT;
		}
	ASSERT(false, "Sobel Scharr Error: unhandled border mode");
	return CONSTANT;
	}

SobelScharr::DepthModeHelper::DepthModeHelper(const std::string& parameterName, DepthMode& depthVariable, const DepthMode& defaultValue) :
	ParameterHelper(parameterName, depthVariable, defaultValue)
	{

	}

SobelScharr::DepthMode SobelScharr::DepthModeHelper::Convert(const std::string& outputFormat)
	{
	if (outputFormat == "Source" || outputFormat == "0")
		{
		return SOURCE;
		}
	else if (outputFormat == "Signed16" || outputFormat == "1")
		{
		return SIGNED16;
		}
	else if (outputFormat == "Float32" || outputFormat == "2")
		{
		return FLOAT32;
		}
	else if (outputFormat == "Float64" || outputFormat == "3")
		{
		return FLOAT64;
		}
	ASSERT(false, "Sobel Scharr Error: unhandled depth mode for 8-bit source image");

	return SIGNED16;
	}

const SobelScharr::SobelScharrOptionsSet SobelScharr::DEFAULT_PARAMETERS =
	{
	.constantBorderValue = 0,
	.borderMode = CONSTANT,
	.depthMode = SIGNED16,
	.sobelParameters =
		{
		.scale = 1.0,
		.delta = 0
		}
	
     };


cv::Mat SobelScharr::sobelx(cv::Mat inputImage)
{
	int borderMode;
 switch(parameters.borderMode)
  {
	case CONSTANT: borderMode = cv::BORDER_CONSTANT; break;
	case WRAP: borderMode = cv::BORDER_WRAP; break;
	case REFLECT: borderMode = cv::BORDER_REFLECT_101; break;
	case BORDER_DEFAULT: borderMode = cv::BORDER_REFLECT_101; break;
	default: ASSERT(false, " Sobel Schar Derivative: Unhandled border mode");
   } 

	int depthMode;
 switch(parameters.depthMode)
  {
	case SOURCE: depthMode = -1; break;
	case SIGNED16: depthMode = CV_16S; break;
	case FLOAT32: depthMode = CV_32F; break;
	case FLOAT64: depthMode = CV_64F; break;
	case DEFAULT: depthMode = CV_16S ; break;
	default: ASSERT(false, " Sobel Schar Derivative: Unhandled depth mode");
   } 

  double param_scale=(double)parameters.sobelParameters.scale; 
  double param_delta=(double)parameters.sobelParameters.delta; 

  cv::Mat grad_x, abs_grad_x;
  
  cv::Scharr(inputImage, grad_x, depthMode, 1, 0, param_scale, param_delta,borderMode);
  convertScaleAbs(grad_x, abs_grad_x );

 return abs_grad_x;
}

cv::Mat SobelScharr::sobely(cv::Mat inputImage)
{
	int borderMode;
 switch(parameters.borderMode)
  {
	case CONSTANT: borderMode = cv::BORDER_CONSTANT; break;
	case WRAP: borderMode = cv::BORDER_WRAP; break;
	case REFLECT: borderMode = cv::BORDER_REFLECT_101; break;
	case BORDER_DEFAULT: borderMode = cv::BORDER_REFLECT_101; break;
	default: ASSERT(false, " Sobel Schar Derivative: Unhandled border mode");
   } 

	int depthMode;
 switch(parameters.depthMode)
  {
	case SOURCE: depthMode = -1; break;
	case SIGNED16: depthMode = CV_16S; break;
	case FLOAT32: depthMode = CV_32F; break;
	case FLOAT64: depthMode = CV_64F; break;
	case DEFAULT: depthMode = CV_16S ; break;
	default: ASSERT(false, " Sobel Schar Derivative: Unhandled depth mode");
   } 

  double param_scale=(double)parameters.sobelParameters.scale; 
  double param_delta=(double)parameters.sobelParameters.delta; 
 
  cv::Mat grad_y, abs_grad_y;
  
  cv::Scharr(inputImage, grad_y, depthMode, 0, 1, param_scale, param_delta,borderMode);
  convertScaleAbs(grad_y, abs_grad_y );

 return abs_grad_y;
}


void SobelScharr::ValidateParameters()
	{
	ASSERT(parameters.sobelParameters.scale > 0 , "Sobel Scharr derivative Configuration error: scale must be positive");

	}

void SobelScharr::ValidateInputs(cv::Mat inputImage)
	{
	ASSERT(inputImage.type() == CV_8UC1, "Sobel Scharr derivativeerror: input image is not of type CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "Sobel Scharr derivative error: input image is empty");
	ASSERT(!inputImage.empty(), "Sobel Scharr derivative error: input image is empty");
	}

}


/** @} */
