/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "SobelDerivative.hpp"
#include <Frame.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

using namespace FrameWrapper;
using namespace Converters;
using namespace Helpers;

namespace dfn_ci
{

SobelDerivative::SobelDerivative()
{
	parametersHelper.AddParameter<float>("GeneralParameters", "ConstantBorderValue", parameters.constantBorderValue, DEFAULT_PARAMETERS.constantBorderValue);
	parametersHelper.AddParameter<BorderMode, BorderModeHelper>("GeneralParameters", "BorderMode", parameters.borderMode, DEFAULT_PARAMETERS.borderMode);
	parametersHelper.AddParameter<DepthMode, DepthModeHelper>("GeneralParameters", "DepthMode", parameters.depthMode, DEFAULT_PARAMETERS.depthMode);
	parametersHelper.AddParameter<double>("SobelParameters", "Scale", parameters.sobelParameters.scale, DEFAULT_PARAMETERS.sobelParameters.scale);
	parametersHelper.AddParameter<double>("SobelParameters", "Delta", parameters.sobelParameters.delta, DEFAULT_PARAMETERS.sobelParameters.delta);

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
	// Read data from input port
	cv::Mat inputImage = frameToMat.Convert(&inImage);
	ValidateInput(inputImage);

	// Process data
	cv::Mat scharrGradX = sobelx(inputImage);
	cv::Mat scharrGradY = sobely(inputImage);

	// Write data to output ports
	const Frame* tmpX = matToFrame.Convert(scharrGradX);
	Copy(*tmpX, outSobelGradientX);
	delete tmpX;

	const Frame* tmpY = matToFrame.Convert(scharrGradY);
	Copy(*tmpY, outSobelGradientY);
	delete tmpY;
}

SobelDerivative::BorderModeHelper::BorderModeHelper(const std::string& parameterName, BorderMode& boundVariable, const BorderMode& defaultValue)
	: ParameterHelper(parameterName, boundVariable, defaultValue)
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
	ASSERT(false, "Sobel Scharr: error: unhandled border mode");
	return CONSTANT;
}

SobelDerivative::DepthModeHelper::DepthModeHelper(const std::string& parameterName, DepthMode& depthVariable, const DepthMode& defaultValue)
	: ParameterHelper(parameterName, depthVariable, defaultValue)
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
	ASSERT(false, "Sobel Scharr: error: unhandled depth mode for 8-bit source image");
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
		.delta = 0.0
	}
};

cv::Mat SobelDerivative::sobelx(cv::Mat inputImage)
{
	int borderMode;
	switch (parameters.borderMode)
	{
		case CONSTANT:
			borderMode = cv::BORDER_CONSTANT;
			break;
		case WRAP:
			borderMode = cv::BORDER_WRAP;
			break;
		case REFLECT:
			borderMode = cv::BORDER_REFLECT_101;
			break;
		case BORDER_DEFAULT:
			borderMode = cv::BORDER_REFLECT_101;
			break;
		default:
			ASSERT(false, "Sobel Schar Derivative: unhandled border mode");
	}

	int depthMode;
	switch (parameters.depthMode)
	{
		case SOURCE:
			depthMode = -1;
			break;
		case SIGNED16:
			depthMode = CV_16S;
			break;
		case FLOAT32:
			depthMode = CV_32F;
			break;
		case FLOAT64:
			depthMode = CV_64F;
			break;
		case DEFAULT:
			depthMode = CV_16S;
			break;
		default:
			ASSERT(false, "Sobel Schar Derivative: unhandled depth mode");
	}

	double scale = parameters.sobelParameters.scale;
	double delta = parameters.sobelParameters.delta;

	cv::Mat gradientX, abs_gradientX;

	cv::Scharr(inputImage, gradientX, depthMode, 1, 0, scale, delta, borderMode);
	convertScaleAbs(gradientX, abs_gradientX);

	return abs_gradientX;
}

cv::Mat SobelDerivative::sobely(cv::Mat inputImage)
{
	int borderMode;
	switch (parameters.borderMode)
	{
		case CONSTANT:
			borderMode = cv::BORDER_CONSTANT;
			break;
		case WRAP:
			borderMode = cv::BORDER_WRAP;
			break;
		case REFLECT:
			borderMode = cv::BORDER_REFLECT_101;
			break;
		case BORDER_DEFAULT:
			borderMode = cv::BORDER_REFLECT_101;
			break;
		default:
			ASSERT(false, "Sobel Schar Derivative: unhandled border mode");
	}

	int depthMode;
	switch (parameters.depthMode)
	{
		case SOURCE:
			depthMode = -1;
			break;
		case SIGNED16:
			depthMode = CV_16S;
			break;
		case FLOAT32:
			depthMode = CV_32F;
			break;
		case FLOAT64:
			depthMode = CV_64F;
			break;
		case DEFAULT:
			depthMode = CV_16S;
			break;
		default:
			ASSERT(false, "Sobel Schar Derivative: unhandled depth mode");
	}

	double scale = parameters.sobelParameters.scale;
	double delta = parameters.sobelParameters.delta;

	cv::Mat gradientY, abs_gradientY;

	cv::Scharr(inputImage, gradientY, depthMode, 0, 1, scale, delta, borderMode);
	convertScaleAbs(gradientY, abs_gradientY);

	return abs_gradientY;
}

void SobelDerivative::ValidateParameters()
{
	ASSERT(parameters.sobelParameters.scale > 0 ,
		"Sobel Scharr Derivative: configuration error: scale must be strictly positive");
}

void SobelDerivative::ValidateInput(cv::Mat inputImage)
{
	ASSERT(inputImage.type() == CV_8UC1 || inputImage.type() == CV_8UC3,
		"Sobel Scharr Derivative: error: input image must be of type CV_8UC1 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0,
		"Sobel Scharr Derivative: error: input image cannot be empty");
	ASSERT(!inputImage.empty(),
		"Sobel Scharr Derivative: error: input image cannot be empty");
}

}

/** @} */
