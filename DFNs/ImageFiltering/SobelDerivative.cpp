/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "SobelDerivative.hpp"
#include <Types/CPP/Frame.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

using namespace FrameWrapper;
using namespace Converters;
using namespace Helpers;

namespace CDFF
{
namespace DFN
{
namespace EdgeDetection
{

SobelDerivative::SobelDerivative()
{
	parametersHelper.AddParameter<float>("GeneralParameters", "ConstantBorderValue", parameters.constantBorderValue, DEFAULT_PARAMETERS.constantBorderValue);
	parametersHelper.AddParameter<BorderMode, BorderModeHelper>("GeneralParameters", "BorderMode", parameters.borderMode, DEFAULT_PARAMETERS.borderMode);
	parametersHelper.AddParameter<DepthMode, DepthModeHelper>("GeneralParameters", "DepthMode", parameters.depthMode, DEFAULT_PARAMETERS.depthMode);
	parametersHelper.AddParameter<double>("SobelParameters", "Scale", parameters.sobelParameters.scale, DEFAULT_PARAMETERS.sobelParameters.scale);
	parametersHelper.AddParameter<double>("SobelParameters", "Delta", parameters.sobelParameters.delta, DEFAULT_PARAMETERS.sobelParameters.delta);
	parametersHelper.AddParameter<int>("SobelParameters", "KernelSize", parameters.sobelParameters.kernelSize, DEFAULT_PARAMETERS.sobelParameters.kernelSize);
	parametersHelper.AddParameter<DerivativeDirection, DerivativeDirectionHelper>("GeneralParameters", "DerivativeDirection", parameters.derivativeDirection, DEFAULT_PARAMETERS.derivativeDirection);

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
	cv::Mat gradient;
	if (parameters.derivativeDirection == HORIZONTAL)
		{
		gradient = sobelx(inputImage);
		}
	else
		{
		gradient = sobely(inputImage);
		}

	// Write data to output ports
	const Frame* temporary = matToFrame.Convert(gradient);
	Copy(*temporary, outImage);
	delete temporary;
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

SobelDerivative::DerivativeDirectionHelper::DerivativeDirectionHelper(const std::string& parameterName, DerivativeDirection& directionVariable, const DerivativeDirection& defaultValue)
	: ParameterHelper(parameterName, directionVariable, defaultValue)
{
}

SobelDerivative::DerivativeDirection SobelDerivative::DerivativeDirectionHelper::Convert(const std::string& direction)
{
	if (direction == "Horizontal" || direction == "0" || direction == "horizontal")
	{
		return HORIZONTAL;
	}
	else if (direction == "Vertical" || direction == "1" || direction == "vertical")
	{
		return VERTICAL;
	}
	ASSERT(false, "Sobel Scharr: error: unhandled derivative direction, it must be one of {Horizontal, Vertical}");
	return HORIZONTAL;
}

const SobelDerivative::SobelDerivativeOptionsSet SobelDerivative::DEFAULT_PARAMETERS =
{
	/*.constantBorderValue =*/ 0,
	/*.borderMode =*/ CONSTANT,
	/*.depthMode =*/ SIGNED16,
	/*.sobelParameters =*/
	{
		.scale = 1.0,
		.delta = 0.0,
		.kernelSize = 3
	},
	/*.derivativeDirection =*/ HORIZONTAL
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
	int kernelSize =  parameters.sobelParameters.kernelSize;

	cv::Mat gradientX, abs_gradientX;
        if(kernelSize == 3)
	  cv::Scharr(inputImage, gradientX, depthMode, 1, 0, scale, delta, borderMode);
	else
	  cv::Sobel(inputImage, gradientX, depthMode, 1, 0, kernelSize, scale, delta, borderMode);
	cv::convertScaleAbs(gradientX, abs_gradientX);

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
	int kernelSize =  parameters.sobelParameters.kernelSize;

	cv::Mat gradientY, abs_gradientY;
	
	if(kernelSize == 3)
	  cv::Scharr(inputImage, gradientY, depthMode, 0, 1, scale, delta, borderMode);
	else
	  cv::Sobel(inputImage, gradientY, depthMode, 0, 1, kernelSize, scale, delta, borderMode);
	
	cv::convertScaleAbs(gradientY, abs_gradientY);

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
}
}

/** @} */
