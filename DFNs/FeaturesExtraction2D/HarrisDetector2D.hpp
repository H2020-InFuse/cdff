/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESEXTRACTION2D_HARRISDETECTOR2D_HPP
#define FEATURESEXTRACTION2D_HARRISDETECTOR2D_HPP

#include "FeaturesExtraction2DInterface.hpp"
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <Converters/FrameToMatConverter.hpp>
#include <Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace CDFF
{
namespace DFN
{
namespace FeaturesExtraction2D
{
	/**
	 * Extraction of keypoints in a 2D image using Harris-Stephens Corner
	 * Detector. Keypoints are returned without any descriptor.
	 *
	 * Processing steps: grayscaling, Gaussian filtering, computation and
	 * normalization of the structure tensor (Harris-Stephens matrix),
	 * extraction of interest points from the tensor (Harris-Stephens points).
	 *
	 * @param generalParameters.apertureSize
	 * @param generalParameters.blockSize
	 * @param generalParameters.parameterK
	 * @param generalParameters.detectionThreshold
	 * @param generalParameters.useGaussianBlur
	 *        enable or disable Gaussian filtering
	 * @param gaussianBlurParameters
	 *        parameters of the Gaussian filter: width and height of the kernel
	 *        or standard deviation of width and height
	 */
	class HarrisDetector2D : public FeaturesExtraction2DInterface
	{
		public:

			HarrisDetector2D();
			virtual ~HarrisDetector2D();

			virtual void configure();
			virtual void process();

		private:

			struct GaussianBlurOptionsSet
			{
				int kernelWidth;
				int kernelHeight;
				float widthStandardDeviation;
				float heightStandardDeviation;
			};

			struct GeneralOptionsSet
			{
				int apertureSize;
				int blockSize;
				float parameterK;
				int detectionThreshold;
				bool useGaussianBlur;
			};

			struct HarrisOptionsSet
			{
				GeneralOptionsSet generalParameters;
				GaussianBlurOptionsSet gaussianBlurParameters;
			};

			Helpers::ParametersListHelper parametersHelper;
			HarrisOptionsSet parameters;
			static const HarrisOptionsSet DEFAULT_PARAMETERS;

			Converters::FrameToMatConverter frameToMat;
			Converters::MatToVisualPointFeatureVector2DConverter matToVisualPointFeatureVector2D;

			cv::Mat ComputeHarrisImage(cv::Mat inputImage);
			cv::Mat ExtractHarrisPoints(cv::Mat harrisImage);

			void ValidateParameters();
			void ValidateInputs(cv::Mat inputImage);

			void Configure(const YAML::Node& configurationNode);
	};
}
}
}

#endif // FEATURESEXTRACTION2D_HARRISDETECTOR2D_HPP

/** @} */
