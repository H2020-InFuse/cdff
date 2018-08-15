/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef ORBDESCRIPTOR_HPP
#define ORBDESCRIPTOR_HPP

#include "FeaturesDescription2DInterface.hpp"
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <yaml-cpp/yaml.h>

namespace CDFF
{
namespace DFN
{
namespace FeaturesDescription2D
{
	/**
	 * Computation of descriptors for input 2D keypoints using ORB (rotated
	 * BRIEF descriptor).
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
	class OrbDescriptor : public FeaturesDescription2DInterface
	{
		public:

			OrbDescriptor();
			virtual ~OrbDescriptor();

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

			cv::Mat ComputeOrbFeatures(cv::Mat inputImage, std::vector<cv::KeyPoint> keypointsVector);
			std::vector<cv::KeyPoint> Convert(const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& featuresVector);

			void ValidateParameters();
			void ValidateInputs(cv::Mat inputImage, std::vector<cv::KeyPoint> keypointsVector);

			static int ConvertToScoreType(std::string scoreType);
	};
}
}
}

#endif // ORBDESCRIPTOR_HPP

/** @} */
