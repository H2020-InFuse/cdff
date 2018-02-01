/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file OrbDetectorDescriptor.hpp
 * @date 23/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implements ORB (FAST Detector + BRIEF descriptor) for 2d images.
 *  
 *
 * @{
 */

#ifndef ORB_DETECTOR_DESCRIPTOR_HPP
#define ORB_DETECTOR_DESCRIPTOR_HPP

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
#include <opencv2/features2d.hpp>
#include <Helpers/ParametersListHelper.hpp>

namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class OrbDetectorDescriptor : public FeaturesExtraction2DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            OrbDetectorDescriptor();
            ~OrbDetectorDescriptor();
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
		
		struct OrbOptionsSet
			{
			int edgeThreshold;
			int fastThreshold;
			int firstLevel;
			int maxFeaturesNumber;
			int levelsNumber;
			int patchSize;
			double scaleFactor;
			int scoreType;
			int sizeOfBrightnessTestSet;
			};

		Helpers::ParametersListHelper parametersHelper;
		OrbOptionsSet parameters;
		static const OrbOptionsSet DEFAULT_PARAMETERS;

		cv::Mat ComputeOrbFeatures(cv::Mat inputImage);

		void ValidateParameters();
		void ValidateInputs(cv::Mat inputImage);

		static int ConvertToScoreType(std::string scoreType);
    };
}
#endif
/* OrbDetectorDescriptor.hpp */
/** @} */
