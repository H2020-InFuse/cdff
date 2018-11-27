/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef SOBELDERIVATIVE_HPP
#define SOBELDERIVATIVE_HPP

#include "EdgeDetectionInterface.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include <Converters/FrameToMatConverter.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/core/core.hpp>
#include <yaml-cpp/yaml.h>

namespace CDFF
{
namespace DFN
{
namespace EdgeDetection
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
	class SobelDerivative : public EdgeDetectionInterface
	{
		public:

			SobelDerivative();
			virtual ~SobelDerivative();

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

			struct SobelParameters
			{
				double scale;
				double delta;
				int kernelSize;
			};

			struct SobelDerivativeOptionsSet
			{
				float constantBorderValue;
				BorderMode borderMode;
				DepthMode depthMode;
				SobelParameters sobelParameters;
			};

			Helpers::ParametersListHelper parametersHelper;
			SobelDerivativeOptionsSet parameters;
			static const SobelDerivativeOptionsSet DEFAULT_PARAMETERS;

			Converters::FrameToMatConverter frameToMat;
			Converters::MatToFrameConverter matToFrame;

			cv::Mat sobelx(cv::Mat inputImage);
			cv::Mat sobely(cv::Mat inputImage);

			void ValidateParameters();
			void ValidateInput(cv::Mat inputImage);

			void Configure(const YAML::Node& configurationNode);
	};
}
}
}

#endif // SOBELDERIVATIVE_HPP

/** @} */
