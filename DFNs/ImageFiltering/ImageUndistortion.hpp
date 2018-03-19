/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ImageUndistortion.hpp
 * @date 19/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  @brief This DFN executes an image undistotion operation.
 *  
 * This DFN implementation applies the undistortion filter provided by OpenCV.
 * 
 * This DFN implementation requires the following parameters:
 * @param cameraMatrix, this represents the internal parameters of the camera: focal lengths and principle points.
 * @param distortionParametersSet, this represents the distortion coefficients of the camera.
 *
 * @{
 */

#ifndef IMAGE_UNDISTORTION_HPP
#define IMAGE_UNDISTORTION_HPP

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
    class ImageUndistortion : public ImageFilteringInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            ImageUndistortion();
            ~ImageUndistortion();
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

		struct ImageUndistortionOptionsSet
			{
			CameraMatrix cameraMatrix;
			DistortionParametersSet distortionParametersSet;
			};

		Helpers::ParametersListHelper parametersHelper;
		ImageUndistortionOptionsSet parameters;
		static const ImageUndistortionOptionsSet DEFAULT_PARAMETERS;

		cv::Mat Undistort(cv::Mat inputImage);

		void ValidateParameters();
		void ValidateInputs(cv::Mat inputImage);

		void Configure(const YAML::Node& configurationNode);
    };
}
#endif
/* ImageUndistortion.hpp */
/** @} */
