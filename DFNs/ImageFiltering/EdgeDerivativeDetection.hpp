/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef EDGEDERIVATIVEDETECTION_HPP
#define EDGEDERIVATIVEDETECTION_HPP

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
	 * Compute 2D image gradients, this DFN uses standard Sobel operator for detection kernels of dimension NOT equal to 3; this DFN uses the optimized Scharr variant for 
	 * detection kernels of dimension equal to 3 (Scharr optimization performs better than standard Sobel if and only if the kernel dimension is 3).
	 *
	 * @param scale
	 *        optional scale factor for the calculated derivatives
	 * @param delta
	 *        optional delta value that is added to the calculated derivatives
	 * @param borderMode
	 *        how interpolation is performed at the border of the image:
	 *        Constant, Wrap, or Reflect
	 * @param depthMode
	 *        output image depth
	 */
	class EdgeDerivativeDetection : public ImageFilteringInterface
	{
		public:

			EdgeDerivativeDetection();
			virtual ~EdgeDerivativeDetection();

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

			struct EdgeDerivativeDetectionOptionsSet
			{
				float constantBorderValue;
				BorderMode borderMode;
				DepthMode depthMode;
				ConvolutionParameters convolutionParameters;
				DerivativeDirection derivativeDirection;
			};

			Helpers::ParametersListHelper parametersHelper;
			EdgeDerivativeDetectionOptionsSet parameters;
			static const EdgeDerivativeDetectionOptionsSet DEFAULT_PARAMETERS;

			Converters::FrameToMatConverter frameToMat;
			Converters::MatToFrameConverter matToFrame;

			cv::Mat ComputeHorizontalDerivative(cv::Mat inputImage);
			cv::Mat ComputeVerticalDerivative(cv::Mat inputImage);

			void ValidateParameters();
			void ValidateInput(cv::Mat inputImage);

			int ConvertBorderModeToCvBorderMode(BorderMode borderMode);
			int ConvertDepthModeToCvDepthMode(DepthMode depthMode);

			void Configure(const YAML::Node& configurationNode);
	};
}
}
}

#endif // EDGEDERIVATIVEDETECTION_HPP

/** @} */
