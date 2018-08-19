/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEFILTERING_IMAGEUNDISTORTIONRECTIFICATION_HPP
#define IMAGEFILTERING_IMAGEUNDISTORTIONRECTIFICATION_HPP

#include "ImageFilteringInterface.hpp"

#include <Frame.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace CDFF
{
namespace DFN
{
namespace ImageFiltering
{
	/**
	 * Undistort and rectify a 2D image (two-in-one filter provided by OpenCV).
	 *
	 * This DFN implementation applies a pre-computed undistortion and
	 * rectification transformation, provided by OpenCV, to a 2D image.
	 *
	 * The transformation can be provided to this DFN implementation in two
	 * different ways:
	 *
	 * (i)  It can be loaded from a file.
	 * (ii) It can be computed from the intrisic camera parameters
	 *      that are available in the DFN configuration file.
	 *
	 * Since a digital image is a discretized representation of reality,
	 * interpolation is used to apply the transformation to arbitrary points
	 * of interest.
	 *
	 * @param interpolationMethod
	 *        Your choice of interpolation algorithm:
	 *        Nearest, Linear, Cubic, or Lanczos
	 * @param borderMode
	 *        How interpolation is handled at the border of the image:
	 *        Constant, Wrap, or Reflect
	 * @param constantBorderValue
	 *        Value of the border for interpolation with borderMode=Constant
	 *
	 * @param cameraConfigurationMode
	 *        How the undistortion-and-rectification transformation is provided:
	 *        External (file) or Internal (intrisic camera parameters)
	 *
	 * @param transformMapsFilePath
	 *        If cameraConfigurationMode=External,
	 *        path to the file where the transformation is defined
	 *
	 * @param cameraMatrix
	 *        If cameraConfigurationMode=Internal,
	 *        internal parameters of the camera:
	 *        focal lengths and principle points
	 * @param distortionParametersSet
	 *        If cameraConfigurationMode=Internal,
	 *        distortion coefficients of the camera
	 * @param imageSize
	 *        if cameraConfigurationMode=Internal
	 *        size of the images produced by the camera and handled by the filter
	 * @param rectificationMatrix
	 *        if cameraConfigurationMode=Internal
	 *        3-by-3 rectification matrix, in the form of a 9-element vector
	 */
	class ImageUndistortionRectification : public ImageFilteringInterface
	{
		public:

			ImageUndistortionRectification();
			virtual ~ImageUndistortionRectification();

			virtual void configure();
			virtual void process();

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
				float k1, k2, k3, k4, k5, k6;
				float p1, p2;
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

			Converters::FrameToMatConverter frameToMat;
			Converters::MatToFrameConverter matToFrame;

			cv::Mat UndistortAndRectify(cv::Mat inputImage);

			void ValidateParameters();
			void ValidateInputs(cv::Mat inputImage);

			void Configure(const YAML::Node& configurationNode);
			void ConvertParametersToCvMatrices();
			void ComputeUndistortionRectificationMap();
	};
}
}
}

#endif // IMAGEFILTERING_IMAGEUNDISTORTIONRECTIFICATION_HPP

/** @} */
