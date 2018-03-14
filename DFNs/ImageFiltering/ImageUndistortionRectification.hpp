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
		enum CameraConfigurationMode
			{
			IN_CONFIGURATION_FILE,
			EXTERNAL_RECTIFICATION_TRANSFORM
			};
		class CameraConfigurationModeHelper : public Helpers::ParameterHelper<CameraConfigurationMode, std::string>
			{
			public:
				CameraConfigurationModeHelper(const std::string& parameterName, CameraConfigurationMode& boundVariable, const CameraConfigurationMode& defaultValue);
			private:
				CameraConfigurationMode Convert(const std::string& value);
			};

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

		struct CameraMatrix
			{
			float focalLengthX;
			float focalLengthY;
			float principlePointX;
			float principlePointY;
			};
		
		struct DistortionParametersSet
			{
			float k1;
			float k2;
			float k3;
			float k4;
			float k5;
			float k6;
			float p1;
			float p2;
			bool useK3;
			bool useK4ToK6;
			};

		struct ImageSize
			{
			int width;
			int height;
			};

		typedef float RectificationMatrix[9];

		struct ImageUndistortionRectificationOptionsSet
			{
			InterpolationMethod interpolationMethod;
			BorderMode borderMode;
			float constantBorderValue;
			std::string transformMapsFilePath;
			CameraConfigurationMode cameraConfigurationMode;

			CameraMatrix cameraMatrix;
			DistortionParametersSet distortionParametersSet;
			ImageSize imageSize;
			RectificationMatrix rectificationMatrix;
			};

		cv::Mat transformMap1, transformMap2;
		cv::Mat cameraMatrix, distortionVector, rectificationMatrix;

		Helpers::ParametersListHelper parametersHelper;
		ImageUndistortionRectificationOptionsSet parameters;
		static const ImageUndistortionRectificationOptionsSet DEFAULT_PARAMETERS;
		void LoadUndistortionRectificationMaps();

		cv::Mat UndistortAndRectify(cv::Mat inputImage);

		void ValidateParameters();
		void ValidateInputs(cv::Mat inputImage);

		void Configure(const YAML::Node& configurationNode);
		void ConvertParametersToCvMatrices();
		void ComputeUndistortionRectificationMap();
    };
}
#endif
/* ImageUndistortionRectification.hpp */
/** @} */
