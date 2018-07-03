/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file SobelDerivative.hpp
 * @date 12/04/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 *
 *  @brief This DFN implements a Sobel image gradient (e.g Scharr derivative).
 *
 * This DFN is Sobel implementation (e.g Scharr operator) of OpenCV .
 *
 * This DFN implementation requires the following parameters:
 * @param scaleDerivative, optional scale factor for the computed derivative values
 * @param delta, optional delta value that is added to the results prior to storing them
 * @param borderMode, the way interpolation is handled at the border of the image. One between Constant, Wrap or Reflect
 * @param depthGradImage, output image depth
 * @{
 */

#ifndef SOBEL_DERIVATIVE_HPP
#define SOBEL_DERIVATIVE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <EdgeDetection/EdgeDetectionInterface.hpp>
#include <Frame.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include <Helpers/ParametersListHelper.hpp>


namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class SobelDerivative : public EdgeDetectionInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            SobelDerivative();
            ~SobelDerivative();
            void process();
            void configure();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
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
				float scale;
				float delta;

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

		cv::Mat sobelx(cv::Mat inputImage);
		cv::Mat sobely(cv::Mat inputImage);

		void ValidateParameters();
		void ValidateInputs(cv::Mat inputImage);

		void Configure(const YAML::Node& configurationNode);
    };
}
#endif
/* SOBEL_DERIVATIVE */
/** @} */
