/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesExtractionDescriptionMatching.cpp
 * @date 23/01/2019
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * This class is the main class for the performance test of the integration pipeline of the
 * following three DFNs: FeatureExtraction2D, FeatureDescription2D and FeatureMatching2D
 *
 *
 * @{
 */

#ifndef FEATURES_EXTRACTION_DESCRIPTION_MATCHING
#define FEATURES_EXTRACTION_DESCRIPTION_MATCHING

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>

#include <Converters/MatToFrameConverter.hpp>

#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNsIntegration/PerformanceTestInterface.hpp>

#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <Types/CPP/CorrespondenceMap2D.hpp>

class FeaturesExtractionDescriptionMatching : public PerformanceTestInterface
	{
	public:
		FeaturesExtractionDescriptionMatching(const std::string& folderPath, const std::vector<std::string>& baseConfigurationFileNamesList, const std::string& performanceMeasuresFileName);
		~FeaturesExtractionDescriptionMatching();

		void SetInputFiles(const std::string& leftImageFilePath, const std::string& rightImageFilePath);
		void SetDfns(CDFF::DFN::FeaturesExtraction2DInterface* extractor, CDFF::DFN::FeaturesDescription2DInterface* descriptor,
			CDFF::DFN::FeaturesMatching2DInterface* matcher);
	protected:

	private:
		std::string leftImageFilePath, rightImageFilePath;
		cv::Mat cvLeftImage;
		cv::Mat cvRightImage;

		FrameWrapper::FrameConstPtr leftFrame;
		FrameWrapper::FrameConstPtr rightFrame;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr leftFeaturesVectorHolder;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr correspondenceMap;

		CDFF::DFN::FeaturesExtraction2DInterface* extractor = nullptr;
		CDFF::DFN::FeaturesDescription2DInterface* descriptor = nullptr;
		CDFF::DFN::FeaturesMatching2DInterface* matcher = nullptr;

		bool SetNextInputs() override;
		void ExecuteDfns() override;
		MeasuresMap ExtractMeasures() override;
	};

#endif

/* FeaturesExtractionDescriptionMatching.hpp */
/** @} */
