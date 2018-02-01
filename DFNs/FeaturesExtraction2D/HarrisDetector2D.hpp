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
 *  This DFN implements the Harris Detector for 2D Images.
 *  
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
