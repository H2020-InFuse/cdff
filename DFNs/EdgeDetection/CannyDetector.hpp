/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef CANNYDETECTOR_HPP
#define CANNYDETECTOR_HPP

#include "EdgeDetectionInterface.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>

#include <opencv2/core/core.hpp>
#include <yaml-cpp/yaml.h>

namespace dfn_ci
{
	/**
	 * Detect edges in a 2D image, using the Canny edge detector provided by
	 * OpenCV.
	 *
	 * @param lowThreshold
	 * @param highThreshold
	 *        lower and higher threshold parameters of the Canny edge detector
	 * @param kernelSize
	 *        size of the filter e.g. 3 or 5 pixels
	 */
	class CannyDetector : public EdgeDetectionInterface
	{
		public:

			CannyDetector();
			virtual ~CannyDetector();

			virtual void configure();
			virtual void process();

		private:

			struct CannyParameters
			{
				double lowThreshold;
				double highThreshold;
				int kernelSize;
			};

			struct CannyDetectorOptionsSet
			{
				CannyParameters cannyParameters;
			};

			Helpers::ParametersListHelper parametersHelper;
			CannyDetectorOptionsSet parameters;
			static const CannyDetectorOptionsSet DEFAULT_PARAMETERS;

			Converters::FrameToMatConverter frameToMat;
			Converters::MatToFrameConverter matToFrame;

			cv::Mat Canny(cv::Mat inputImage);

			void ValidateParameters();
			void ValidateInput(cv::Mat inputImage);

			void Configure(const YAML::Node& configurationNode);
	};
}

#endif // CANNYDETECTOR_HPP

/** @} */
