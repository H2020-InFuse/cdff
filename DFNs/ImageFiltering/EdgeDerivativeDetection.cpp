/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "EdgeDerivativeDetection.hpp"
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
namespace ImageFiltering
{

EdgeDerivativeDetection::EdgeDerivativeDetection()
{
	parametersHelper.AddParameter<float>("GeneralParameters", "ConstantBorderValue", parameters.constantBorderValue, DEFAULT_PARAMETERS.constantBorderValue);
	parametersHelper.AddParameter<BorderMode, BorderModeHelper>("GeneralParameters", "BorderMode", parameters.borderMode, DEFAULT_PARAMETERS.borderMode);
	parametersHelper.AddParameter<DepthMode, DepthModeHelper>("GeneralParameters", "DepthMode", parameters.depthMode, DEFAULT_PARAMETERS.depthMode);
	parametersHelper.AddParameter<double>("ConvolutionParameters", "Scale", parameters.convolutionParameters.scale, DEFAULT_PARAMETERS.convolutionParameters.scale);
	parametersHelper.AddParameter<double>("ConvolutionParameters", "Delta", parameters.convolutionParameters.delta, DEFAULT_PARAMETERS.convolutionParameters.delta);
	parametersHelper.AddParameter<KernelType, KernelTypeHelper>("ConvolutionParameters", "KernelType", parameters.convolutionParameters.kernelType, DEFAULT_PARAMETERS.convolutionParameters.kernelType);
	parametersHelper.AddParameter<int>("ConvolutionParameters", "KernelSize", parameters.convolutionParameters.kernelSize, DEFAULT_PARAMETERS.convolutionParameters.kernelSize);
	parametersHelper.AddParameter<DerivativeDirection, DerivativeDirectionHelper>("GeneralParameters", "DerivativeDirection", parameters.derivativeDirection, DEFAULT_PARAMETERS.derivativeDirection);

	configurationFilePath = "";
}

EdgeDerivativeDetection::~EdgeDerivativeDetection()
{
}

void EdgeDerivativeDetection::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void EdgeDerivativeDetection::process()
{
	// Read data from input port
	cv::Mat inputImage = frameToMat.Convert(&inImage);
	ValidateInput(inputImage);

	// Process data
	cv::Mat gradient;
	if (parameters.derivativeDirection == HORIZONTAL)
		{
		gradient = ComputeHorizontalDerivative(inputImage);
		}
	else
		{
		gradient = ComputeVerticalDerivative(inputImage);
		}

	// Write data to output ports
	const Frame* temporary = matToFrame.Convert(gradient);
	Copy(*temporary, outImage);
	delete temporary;
}

EdgeDerivativeDetection::BorderModeHelper::BorderModeHelper(const std::string& parameterName, BorderMode& boundVariable, const BorderMode& defaultValue)
	: ParameterHelper(parameterName, boundVariable, defaultValue)
{
}

EdgeDerivativeDetection::BorderMode EdgeDerivativeDetection::BorderModeHelper::Convert(const std::string& outputFormat)
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
	ASSERT(false, "Edge Derivative Detection: error: unhandled border mode");
	return CONSTANT;
}

EdgeDerivativeDetection::DepthModeHelper::DepthModeHelper(const std::string& parameterName, DepthMode& depthVariable, const DepthMode& defaultValue)
	: ParameterHelper(parameterName, depthVariable, defaultValue)
{
}

EdgeDerivativeDetection::DepthMode EdgeDerivativeDetection::DepthModeHelper::Convert(const std::string& outputFormat)
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
	ASSERT(false, "Edge Derivative Detection: error: unhandled depth mode for 8-bit source image");
	return SIGNED16;
}

EdgeDerivativeDetection::KernelTypeHelper::KernelTypeHelper(const std::string& parameterName, KernelType& typeVariable, const KernelType& defaultValue)
	: ParameterHelper(parameterName, typeVariable, defaultValue)
{
}

EdgeDerivativeDetection::KernelType EdgeDerivativeDetection::KernelTypeHelper::Convert(const std::string& kernelType)
{
	if (kernelType == "Sobel" || kernelType == "0")
	{
		return SOBEL;
	}
	else if (kernelType == "Scharr" || kernelType == "1")
	{
		return SCHARR;
	}
	ASSERT(false, "Edge Derivative Detection: error: unhandled kernel type, it must be one of {Sobel, Scharr}");
	return SOBEL;
}

EdgeDerivativeDetection::DerivativeDirectionHelper::DerivativeDirectionHelper(const std::string& parameterName, DerivativeDirection& directionVariable, const DerivativeDirection& defaultValue)
	: ParameterHelper(parameterName, directionVariable, defaultValue)
{
}

EdgeDerivativeDetection::DerivativeDirection EdgeDerivativeDetection::DerivativeDirectionHelper::Convert(const std::string& direction)
{
	if (direction == "Horizontal" || direction == "0" || direction == "horizontal")
	{
		return HORIZONTAL;
	}
	else if (direction == "Vertical" || direction == "1" || direction == "vertical")
	{
		return VERTICAL;
	}
	ASSERT(false, "Edge Derivative Detection: error: unhandled derivative direction, it must be one of {Horizontal, Vertical}");
	return HORIZONTAL;
}

const EdgeDerivativeDetection::EdgeDerivativeDetectionOptionsSet EdgeDerivativeDetection::DEFAULT_PARAMETERS =
{
	/*.constantBorderValue =*/ 0,
	/*.borderMode =*/ CONSTANT,
	/*.depthMode =*/ SIGNED16,
	/*.convolutionParameters =*/
	{
		/*.scale =*/ 1.0,
		/*.delta =*/ 0.0,
		/*.kernelType =*/ SOBEL,
		/*.kernelSize =*/ 3
	},
	/*.derivativeDirection =*/ HORIZONTAL
};

cv::Mat EdgeDerivativeDetection::ComputeHorizontalDerivative(cv::Mat inputImage)
{
	int borderMode = ConvertBorderModeToCvBorderMode(parameters.borderMode);
	int depthMode = ConvertDepthModeToCvDepthMode(parameters.depthMode);

	double scale = parameters.convolutionParameters.scale;
	double delta = parameters.convolutionParameters.delta;
	int kernelSize =  parameters.convolutionParameters.kernelSize;

	cv::Mat gradientX, abs_gradientX;
	if(parameters.convolutionParameters.kernelType == SCHARR)
		{
		cv::Scharr(inputImage, gradientX, depthMode, 1, 0, scale, delta, borderMode);
		}
	else
		{
		cv::Sobel(inputImage, gradientX, depthMode, 1, 0, kernelSize, scale, delta, borderMode);
		}
	cv::convertScaleAbs(gradientX, abs_gradientX);

	return abs_gradientX;
}

cv::Mat EdgeDerivativeDetection::ComputeVerticalDerivative(cv::Mat inputImage)
{
	int borderMode = ConvertBorderModeToCvBorderMode(parameters.borderMode);
	int depthMode = ConvertDepthModeToCvDepthMode(parameters.depthMode);

	double scale = parameters.convolutionParameters.scale;
	double delta = parameters.convolutionParameters.delta;
	int kernelSize =  parameters.convolutionParameters.kernelSize;

	cv::Mat gradientY, abs_gradientY;
	
	if(parameters.convolutionParameters.kernelType == SCHARR)
		{
		cv::Scharr(inputImage, gradientY, depthMode, 0, 1, scale, delta, borderMode);
		}
	else
		{
		cv::Sobel(inputImage, gradientY, depthMode, 0, 1, kernelSize, scale, delta, borderMode);
		}	
	cv::convertScaleAbs(gradientY, abs_gradientY);

	return abs_gradientY;
}

int EdgeDerivativeDetection::ConvertBorderModeToCvBorderMode(BorderMode borderMode)
{
	switch (borderMode)
	{
		case CONSTANT:
			return cv::BORDER_CONSTANT;
		case WRAP:
			return cv::BORDER_WRAP;
		case REFLECT:
			return cv::BORDER_REFLECT_101;
		case BORDER_DEFAULT:
			return cv::BORDER_REFLECT_101;
		default:
			ASSERT(false, "Edge Derivative Detection:: unhandled border mode");
	}
}

int EdgeDerivativeDetection::ConvertDepthModeToCvDepthMode(DepthMode depthMode)
{
	switch (depthMode)
	{
		case SOURCE:
			return -1;
		case SIGNED16:
			return CV_16S;
		case FLOAT32:
			return CV_32F;
		case FLOAT64:
			return CV_64F;
		case DEFAULT:
			return CV_16S;
		default:
			ASSERT(false, "Edge Derivative Detection:: unhandled depth mode");
	}
}

void EdgeDerivativeDetection::ValidateParameters()
{
	ASSERT(parameters.convolutionParameters.scale > 0 ,
		"Edge Derivative Detection configuration error: scale must be strictly positive");
	ASSERT(parameters.convolutionParameters.kernelType != SCHARR || parameters.convolutionParameters.kernelSize == 3, 
		"Edge Derivative Detection error: Scharr kernel type is supported only for kernel size equal to 3");
}

void EdgeDerivativeDetection::ValidateInput(cv::Mat inputImage)
{
	ASSERT(inputImage.type() == CV_8UC1 || inputImage.type() == CV_8UC3,
		"Edge Derivative Detection: error: input image must be of type CV_8UC1 or CV_8UC1");
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0,
		"Edge Derivative Detection: error: input image cannot be empty");
	ASSERT(!inputImage.empty(),
		"Edge Derivative Detection: error: input image cannot be empty");
}

}
}
}

/** @} */
