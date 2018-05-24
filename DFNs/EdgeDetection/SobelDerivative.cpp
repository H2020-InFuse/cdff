/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SobelDerivative.cpp
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
#include "SobelDerivative.hpp"
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
SobelDerivative::SobelDerivative()
	{
	parametersHelper.AddParameter<float>("GeneralParameters", "ConstantBorderValue", parameters.constantBorderValue, DEFAULT_PARAMETERS.constantBorderValue);
	parametersHelper.AddParameter<BorderMode, BorderModeHelper>("GeneralParameters", "BorderMode", parameters.borderMode, DEFAULT_PARAMETERS.borderMode);
	parametersHelper.AddParameter<DepthMode, DepthModeHelper>("GeneralParameters", "DepthMode", parameters.depthMode, DEFAULT_PARAMETERS.depthMode);
	parametersHelper.AddParameter<float>("SobelParameters", "Scale", parameters.sobelParameters.scale, DEFAULT_PARAMETERS.sobelParameters.scale);
	parametersHelper.AddParameter<float>("SobelParameters", "Delta", parameters.sobelParameters.delta, DEFAULT_PARAMETERS.sobelParameters.delta);
	
	configurationFilePath = "";
	}
SobelDerivative::~SobelDerivative()
{

}

void SobelDerivative::configure()
	{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
	}


void SobelDerivative::process() 
	{
	cv::Mat inputImage = ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Convert(inImage);
	ValidateInputs(inputImage);
	
	cv::Mat  scharrGradx= sobelx(inputImage);
	outSobelGradientX = ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Convert(scharrGradx);

	cv::Mat  scharrGrady= sobely(inputImage);
	outSobelGradientY = ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Convert(scharrGrady);
	}


SobelDerivative::BorderModeHelper::BorderModeHelper(const std::string& parameterName, BorderMode& boundVariable, const BorderMode& defaultValue) :
	ParameterHelper(parameterName, boundVariable, defaultValue)
	{

	}

SobelDerivative::BorderMode SobelDerivative::BorderModeHelper::Convert(const std::string& outputFormat)
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

SobelDerivative::DepthModeHelper::DepthModeHelper(const std::string& parameterName, DepthMode& depthVariable, const DepthMode& defaultValue) :
	ParameterHelper(parameterName, depthVariable, defaultValue)
	{

	}

SobelDerivative::DepthMode SobelDerivative::DepthModeHelper::Convert(const std::string& outputFormat)
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

const SobelDerivative::SobelDerivativeOptionsSet SobelDerivative::DEFAULT_PARAMETERS =
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


cv::Mat SobelDerivative::sobelx(cv::Mat inputImage)
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

  cv::Mat gradientX, abs_gradientX;
  
  cv::Scharr(inputImage, gradientX, depthMode, 1, 0, param_scale, param_delta,borderMode);
  convertScaleAbs(gradientX, abs_gradientX );

 return abs_gradientX;
}

cv::Mat SobelDerivative::sobely(cv::Mat inputImage)
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
 
  cv::Mat gradientY, abs_gradientY;
  
  cv::Scharr(inputImage, gradientY, depthMode, 0, 1, param_scale, param_delta,borderMode);
  convertScaleAbs(gradientY, abs_gradientY );

 return abs_gradientY;
}


void SobelDerivative::ValidateParameters()
	{
	ASSERT(parameters.sobelParameters.scale > 0 , "Sobel Scharr derivative Configuration error: scale must be positive");

	}

void SobelDerivative::ValidateInputs(cv::Mat inputImage)
	{
	ASSERT(inputImage.type() == CV_8UC1||inputImage.type() == CV_8UC3, "Sobel Scharr derivative error: input image is not of type CV_8UC1 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "Sobel Scharr derivative error: input image is empty");
	ASSERT(!inputImage.empty(), "Sobel Scharr derivative error: input image is empty");
	}

}


/** @} */
