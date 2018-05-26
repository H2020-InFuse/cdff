/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file OrbDescriptor.hpp
 * @date 21/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  @brief This DFN uses the Orb Descriptor (Rotated BRIEF descriptor) for the computation of the descriptors of the input keypoints.
 *  
 *  The algorithm uses the following parameters:
 *  @param generalParameters.edgeThreshold
 *  @param generalParameters.fastThreshold
 *  @param generalParameters.firstLevel
 *  @param generalParameters.maxFeaturesNumber, this determines the maximum number of keypoints the algorithm will extract.
 *  @param generalParameters.levelsNumber
 *  @param generalParameters.patchSize
 *  @param generalParameters.scaleFactor
 *  @param generalParameters.scoreType
 *  @param generalParameters.sizeOfBrightnessTestSet
 *
 * @{
 */

#ifndef ORBDESCRIPTOR_HPP
#define ORBDESCRIPTOR_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "FeaturesDescription2DInterface.hpp"
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/features2d.hpp>
#include <Helpers/ParametersListHelper.hpp>

namespace dfn_ci
{

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class OrbDescriptor : public FeaturesDescription2DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:

            OrbDescriptor();
            virtual ~OrbDescriptor();

            virtual void configure();
            virtual void process();

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

		Converters::FrameToMatConverter frameToMat;
		Converters::MatToVisualPointFeatureVector2DConverter matToVisualPointFeatureVector2D;

		cv::Mat ComputeOrbFeatures(cv::Mat inputImage, std::vector<cv::KeyPoint> keypointsVector);
		std::vector<cv::KeyPoint> Convert(const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& featuresVector);

		void ValidateParameters();
		void ValidateInputs(cv::Mat inputImage, std::vector<cv::KeyPoint> keypointsVector);

		static int ConvertToScoreType(std::string scoreType);
    };
}

#endif // ORBDESCRIPTOR_HPP

/** @} */
