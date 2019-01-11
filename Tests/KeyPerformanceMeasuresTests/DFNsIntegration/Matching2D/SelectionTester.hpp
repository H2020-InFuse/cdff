/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SelectionTester.hpp
 * @date 07/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class will test requirement 4.1.1.3 of deliverable 5.5.
 * "90% of manually defined matches on manually defined features should be detected correctly.  Manually defined matches are considered to be matches between features made by a 
 * human on close inspection of a set of features on a pair of images.  A human will inspect each set of matches and identify those that are incorrect."
 *
 * Requirement 4.1.1.3 has also another statement, but this one is the one that should be verified by the Requirement 4.1.1.2 since it refers to the detection function only.
 *
 * @{
 */

#ifndef SELECTION_TESTER_HPP
#define SELECTION_TESTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <Errors/Assert.hpp>

#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <Types/CPP/Frame.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/BaseTypes.hpp>
#include <Types/CPP/CorrespondenceMap2D.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>
#include <boost/make_shared.hpp>

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class SelectionTester
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		SelectionTester();
		~SelectionTester();

		void SetDfns(CDFF::DFN::FeaturesDescription2DInterface* descriptor, CDFF::DFN::FeaturesMatching2DInterface* matcher);
		void SetConfigurationFilePaths(const std::string& featuresDescriptorConfigurationFilePath, const std::string& featuresMatcherConfigurationFilePath);
		void SetInputFilesPaths(const std::string& sourceImageFilePath, const std::string& sinkImageFilePath, const std::string& correspondencesImageFilePath);
		void ExecuteDfns();
		bool AreCorrespondencesValid(float percentageThreshold);

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
		std::string featuresDescriptorConfigurationFilePath;
		std::string featuresMatcherConfigurationFilePath;
		std::string sourceImageFilePath;
		std::string sinkImageFilePath;
		std::string correspondencesImageFilePath;

		FrameWrapper::FrameConstPtr inputSourceFrame, inputSinkFrame;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputSourceKeypointsVector, inputSinkKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr sourceFeaturesVector, sinkFeaturesVector;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr outputCorrespondenceMap;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr referenceCorrespondenceMap;

		Converters::MatToFrameConverter frameConverter;
		CDFF::DFN::FeaturesDescription2DInterface* descriptor;
		CDFF::DFN::FeaturesMatching2DInterface* matcher;

		bool dfnsWereLoaded;
		bool inputImagesWereLoaded;
		bool inputKeypointsWereLoaded;
		bool precisionReferenceWasLoaded;

		void LoadInputImage(const std::string& imageFilePath, FrameWrapper::FrameConstPtr& frame);
		void LoadReferenceCorrespondenceMap();
		void ConfigureDfns();

		bool ValidateCorrespondences(float percentageThreshold);
		bool CorrespondencesAreTheSame(int referenceIndex, int outputIndex);
	};

#endif

/* SelectionTester.hpp */
/** @} */
