/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file CannyDetector.hpp
 * @date 11/04/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 *
 *  @brief This DFN executes a Canny edge detection.
 *
 * This DFN is Canny edge detector implementation of OpenCV.
 *
 * This DFN implementation requires the following parameters:
 * @param lowThreshold, lower threshold parameter of the canny edge detector.
 * @param highThreshold, higher threshold parameter.
 * @param kernelSize, size of the filter e.g 3, 5.
 *
 * @{
 */

#ifndef CANNY_DETECTOR_HPP
#define CANNY_DETECTOR_HPP

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
    class CannyDetector : public EdgeDetectionInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            CannyDetector();
            ~CannyDetector();
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

		struct CannyParameters
			{
				float lowThreshold;
				float highThreshold;
				float kernelSize;
			};


		struct CannyDetectorOptionsSet
			{
			  CannyParameters cannyParameters;

			};

		Helpers::ParametersListHelper parametersHelper;
		CannyDetectorOptionsSet parameters;

		static const CannyDetectorOptionsSet DEFAULT_PARAMETERS;

		cv::Mat Canny(cv::Mat inputImage);

		void ValidateParameters();
		void ValidateInputs(cv::Mat inputImage);

		void Configure(const YAML::Node& configurationNode);
    };
}
#endif
/* CANNY_DETECTOR.hpp */
/** @} */
