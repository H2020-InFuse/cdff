/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DERIVATIVEEDGEDETECTION_HPP
#define DERIVATIVEEDGEDETECTION_HPP

#include "ImageFilteringInterface.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include <Converters/FrameToMatConverter.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/core/core.hpp>
#include <yaml-cpp/yaml.h>

namespace CDFF
{
namespace DFN
{
namespace ImageFiltering
{
	/**
	 * Compute 2D image gradients, this DFN uses either the Sobel or the Scharr operators.
	 *
	 * @param convolutionParameters.scale
	 *        optional scale factor for the calculated derivatives;
	 * @param convolutionParameters.delta
	 *        optional delta value that is added to the calculated derivatives;
	 * @param convolutionParameters.kernelType
	 *	  the kernel type used in the convolution, this parameter takes values in {Sobel, Scharr};
	 * @param convolutionParameters.kernelSize
	 *	  the size of the kernel, Scharr is supported only for kernel size equal to 3;
	 * @param borderMode
	 *        how interpolation is performed at the border of the image:
	 *        this parameter takes values in  {Constant, Wrap, Reflect, BorderDefault};
	 * @param depthMode
	 *        output image depth, this parameter takes values in  {Source, Signed16, Float32, Float64};
	 * @param derivativeDirection
	 *	  whether the operator should be applied along the x or y axis; this parameter takes values in {Horizontal, Vertical}.
	 */
	class DerivativeEdgeDetection : public ImageFilteringInterface
	{
		public:

			DerivativeEdgeDetection();
			virtual ~DerivativeEdgeDetection();

			virtual void configure();
			virtual void process();

		private:

			enum BorderMode
			{
				CONSTANT,
				WRAP,
				REFLECT,
				BORDER_DEFAULT
			};
			class BorderModeHelper : public Helpers::ParameterHelper<BorderMode, std::string>
			{
				public:
					BorderModeHelper(const std::string& parameterName, BorderMode& boundVariable, const BorderMode& defaultValue);
				private:
					BorderMode Convert(const std::string& value);
			};

			enum DepthMode
			{
				SOURCE,
				SIGNED16,
				FLOAT32,
				FLOAT64,
				DEFAULT
			};
			class DepthModeHelper : public Helpers::ParameterHelper<DepthMode, std::string>
			{
				public:
					DepthModeHelper(const std::string& parameterName, DepthMode& depthVariable, const DepthMode& defaultValue);
				private:
					DepthMode Convert(const std::string& value);
			};

			enum KernelType
			{
				SOBEL,
				SCHARR
			};
			class KernelTypeHelper : public Helpers::ParameterHelper<KernelType, std::string>
			{
				public:
					KernelTypeHelper(const std::string& parameterName, KernelType& depthVariable, const KernelType& defaultValue);
				private:
					KernelType Convert(const std::string& value);
			};

			struct ConvolutionParameters
			{
				double scale;
				double delta;
				KernelType kernelType;
				int kernelSize;
			};

			enum DerivativeDirection
			{
				HORIZONTAL,
				VERTICAL
			};
			class DerivativeDirectionHelper : public Helpers::ParameterHelper<DerivativeDirection, std::string>
			{
				public:
					DerivativeDirectionHelper(const std::string& parameterName, DerivativeDirection& depthVariable, const DerivativeDirection& defaultValue);
				private:
					DerivativeDirection Convert(const std::string& value);
			};

			struct DerivativeEdgeDetectionOptionsSet
			{
				float constantBorderValue;
				BorderMode borderMode;
				DepthMode depthMode;
				ConvolutionParameters convolutionParameters;
				DerivativeDirection derivativeDirection;
			};

			Helpers::ParametersListHelper parametersHelper;
			DerivativeEdgeDetectionOptionsSet parameters;
			static const DerivativeEdgeDetectionOptionsSet DEFAULT_PARAMETERS;

			Converters::FrameToMatConverter frameToMat;
			Converters::MatToFrameConverter matToFrame;

			cv::Mat ComputeDerivative(cv::Mat inputImage);

			void ValidateParameters();
			void ValidateInput(cv::Mat inputImage);

			int ConvertBorderModeToCvBorderMode(BorderMode borderMode);
			int ConvertDepthModeToCvDepthMode(DepthMode depthMode);

			void Configure(const YAML::Node& configurationNode);
	};
}
}
}

#endif // DERIVATIVEEDGEDETECTION_HPP

/** @} */
