/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file HarrisDetector2D.hpp
 * @date 17/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  @brief This DFN uses the Harris Detector for detection of keypoints in 2D images. It does not provide a descriptor.
 *
 *  The DFN operates according the following steps: transformation of the image into grey scale, application of a Gaussian Filter, computation and normalization of the Harris Matrix, and extraction of
 *  the Harris points from the matrix.
 *
 *  The algorithm uses the following parameters:
 *  @param generalParameters.apertureSize
 *  @param generalParameters.blockSize
 *  @param generalParameters.parameterK
 *  @param generalParameters.detectionThreshold
 *  @param generalParameters.useGaussianBlur, this parameters determines whether the application of the Gaussian filter should be skipped or not.
 *  @param gaussianBlurParameters, this contains the parameters of the Gaussian filter, either as width and height of the kernel or as standard deviation of width and height.
 *
 * @{
 */

#ifndef HARRIS_DETECTOR_2D_HPP
#define HARRIS_DETECTOR_2D_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Helpers/ParametersListHelper.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>


namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class HarrisDetector2D : public FeaturesExtraction2DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            HarrisDetector2D();
            ~HarrisDetector2D();
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

		struct HarryOptionsSet
			{
			GeneralOptionsSet generalParameters;
			GaussianBlurOptionsSet gaussianBlurParameters;
			};

		Helpers::ParametersListHelper parametersHelper;
		HarryOptionsSet parameters;
		static const HarryOptionsSet DEFAULT_PARAMETERS;

		Converters::FrameToMatConverter frameToMat;
		Converters::MatToVisualPointFeatureVector2DConverter matToVisualPointFeatureVector2D;

		cv::Mat ComputeHarrisImage(cv::Mat inputImage);
		cv::Mat ExtractHarrisPoints(cv::Mat harrisImage);

		void ValidateParameters();
		void ValidateInputs(cv::Mat inputImage);

		void Configure(const YAML::Node& configurationNode);
    };
}
#endif
/* HarrisDetector2D.hpp */
/** @} */
