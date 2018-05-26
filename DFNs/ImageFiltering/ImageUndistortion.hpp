/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEUNDISTORTION_HPP
#define IMAGEUNDISTORTION_HPP

#include "ImageFilteringInterface.hpp"

#include <Frame.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace dfn_ci
{
	/**
	 * Undistort a 2D image (filter provided by OpenCV).
	 *
	 * @param cameraMatrix
	 *        Internal parameters of the camera:
	 *        focal lengths and principle points
	 * @param distortionParametersSet
	 *        Distortion coefficients of the camera
	 */
	class ImageUndistortion : public ImageFilteringInterface
	{
		public:

			ImageUndistortion();
			virtual ~ImageUndistortion();

			virtual void configure();
			virtual void process();

	private:

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

		struct ImageUndistortionOptionsSet
		{
			CameraMatrix cameraMatrix;
			DistortionParametersSet distortionParametersSet;
		};

		Helpers::ParametersListHelper parametersHelper;
		ImageUndistortionOptionsSet parameters;
		static const ImageUndistortionOptionsSet DEFAULT_PARAMETERS;

		Converters::FrameToMatConverter frameToMat;
		Converters::MatToFrameConverter matToFrame;

		cv::Mat Undistort(cv::Mat inputImage);

		void ValidateParameters();
		void ValidateInputs(cv::Mat inputImage);

		void Configure(const YAML::Node& configurationNode);
	};
}

#endif // IMAGEUNDISTORTION_HPP

/** @} */
