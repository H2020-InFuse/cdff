/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef ORBDETECTORDESCRIPTOR_HPP
#define ORBDETECTORDESCRIPTOR_HPP

#include "FeaturesExtraction2DInterface.hpp"
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <yaml-cpp/yaml.h>

namespace dfn_ci
{
	/**
	 * Extraction of keypoints from a 2D image using ORB (modified FAST detector
	 * and BRIEF descriptor). Descriptors are returned.
	 *
	 * @param generalParameters.edgeThreshold
	 * @param generalParameters.fastThreshold
	 * @param generalParameters.firstLevel
	 * @param generalParameters.maxFeaturesNumber
	 *        maximal number of keypoints that the algorithm will extract
	 * @param generalParameters.levelsNumber
	 * @param generalParameters.patchSize
	 * @param generalParameters.scaleFactor
	 * @param generalParameters.scoreType
	 * @param generalParameters.sizeOfBrightnessTestSet
	 */
	class OrbDetectorDescriptor : public FeaturesExtraction2DInterface
	{
		public:

			OrbDetectorDescriptor();
			virtual ~OrbDetectorDescriptor();

			virtual void configure();
			virtual void process();

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

			cv::Mat ComputeOrbFeatures(cv::Mat inputImage);

			void ValidateParameters();
			void ValidateInputs(cv::Mat inputImage);

			static int ConvertToScoreType(std::string scoreType);
	};
}

#endif // ORBDETECTORDESCRIPTOR_HPP

/** @} */
