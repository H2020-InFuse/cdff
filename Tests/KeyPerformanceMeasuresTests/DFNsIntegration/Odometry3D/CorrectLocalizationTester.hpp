/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CorrectLocalizationTester.hpp
 * @date 03/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class will test requirement 4.1.1.9 of deliverable 5.5.
 * " The matched model within the scene should be located and oriented within 10% of the size and relative orientation of the model. " and
 * " Distance between ground truth pose and estimate pose should not be higher than 5% of the operating distance."
 *
 * @{
 */

#ifndef CORRECT_LOCALIZATION_TESTER_HPP
#define CORRECT_LOCALIZATION_TESTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>
#include <Errors/Assert.hpp>
#include <ctime>

#include <VisualPointFeatureVector3D.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <SupportTypes.hpp>

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
class CorrectLocalizationTester
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		CorrectLocalizationTester();
		~CorrectLocalizationTester();

		void SetInputClouds(std::string sceneCloudFilePath, std::string modelCloudFilePath, std::string groundTruthPoseFilePath);
		void SetConfigurationFiles(std::string extractorConfigurationFile, std::string descriptorConfigurationFile, std::string matcherConfigurationFile);
		void SetDfns(dfn_ci::FeaturesExtraction3DInterface* extractor, dfn_ci::FeaturesDescription3DInterface* descriptor, dfn_ci::FeaturesMatching3DInterface* matcher);

		void ExecuteDfns();
		bool IsOutputCorrect(float relativeLocationError, float relativeOrientationError, float absoluteLocationError);

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
		std::string extractorConfigurationFile, descriptorConfigurationFile, matcherConfigurationFile;
		std::string sceneCloudFilePath, modelCloudFilePath, groundTruthPoseFilePath;
		dfn_ci::FeaturesExtraction3DInterface* extractor;
		dfn_ci::FeaturesDescription3DInterface* descriptor;
		dfn_ci::FeaturesMatching3DInterface* matcher;

		PointCloudWrapper::PointCloudConstPtr inputSceneCloud;
		PointCloudWrapper::PointCloudConstPtr inputModelCloud;
		PoseWrapper::Pose3DConstPtr inputTruthModelPoseInScene;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sceneKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr modelKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sceneFeaturesVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr modelFeaturesVector;
		PoseWrapper::Pose3DConstPtr outputModelPoseInScene;
		bool outputMatcherSuccess;

		Converters::PclPointCloudToPointCloudConverter pointCloudConverter;
		bool dfnsWereConfigured;
		bool inputsWereLoaded;
		bool groundTruthWasLoaded;
		clock_t beginTime, endTime;
		float processingTime;

		void ExtractFeatures();
		void DescribeFeatures();
		void MatchFeatures();

		void LoadPointClouds();
		void LoadGroudTruthPose();
		void ConfigureDfns();

		float ComputeLocationError();
		float ComputeOrientationError(float modelSize);
		float ComputeModelSize();
	};

#endif

/* CorrectLocalizationTester.hpp */
/** @} */
