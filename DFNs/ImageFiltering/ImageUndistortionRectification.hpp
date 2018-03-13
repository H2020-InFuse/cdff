/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ImageUndistortionRectification.hpp
 * @date 09/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implements both the image undistotion and image rectification operations.
 *  
 *
 * @{
 */

#ifndef IMAGE_UNDISTORTION_RECTIFICATION_HPP
#define IMAGE_UNDISTORTION_RECTIFICATION_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <Frame.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Helpers/ParametersListHelper.hpp>


namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class ImageUndistortionRectification : public ImageFilteringInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            ImageUndistortionRectification();
            ~ImageUndistortionRectification();
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

		enum InterpolationMethod
			{
			NEAREST,
			LINEAR,
			CUBIC,
			LANCZOS
			};
		class InterpolationMethodHelper : public Helpers::ParameterHelper<InterpolationMethod, std::string>
			{
			public:
				InterpolationMethodHelper(const std::string& parameterName, InterpolationMethod& boundVariable, const InterpolationMethod& defaultValue);
			private:
				InterpolationMethod Convert(const std::string& value);
			};

		enum BorderMode
			{
			CONSTANT,
			WRAP,
			REFLECT
			};
		class BorderModeHelper : public Helpers::ParameterHelper<BorderMode, std::string>
			{
			public:
				BorderModeHelper(const std::string& parameterName, BorderMode& boundVariable, const BorderMode& defaultValue);
			private:
				BorderMode Convert(const std::string& value);
			};

		struct ImageUndistortionRectificationOptionsSet
			{
			InterpolationMethod interpolationMethod;
			BorderMode borderMode;
			float constantBorderValue;
			std::string transformMapsFilePath;
			};

		cv::Mat transformMap1, transformMap2;

		Helpers::ParametersListHelper parametersHelper;
		ImageUndistortionRectificationOptionsSet parameters;
		static const ImageUndistortionRectificationOptionsSet DEFAULT_PARAMETERS;
		void LoadTransformMaps();

		cv::Mat UndistortAndRectify(cv::Mat inputImage);

		void ValidateParameters();
		void ValidateInputs(cv::Mat inputImage);

		void Configure(const YAML::Node& configurationNode);
    };
}
#endif
/* ImageUndistortionRectification.hpp */
/** @} */
