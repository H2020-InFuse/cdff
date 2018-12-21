/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CorrectLocalizationTester.hpp
 * @date 05/06/2018
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
#include <Registration3D/Registration3DInterface.hpp>
#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <Errors/Assert.hpp>

#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>

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
		CorrectLocalizationTester(const std::string& configurationFile, CDFF::DFN::Registration3DInterface* dfn, const std::string& extractorConfigurationFile, 
			CDFF::DFN::FeaturesExtraction3DInterface* extractor);
		~CorrectLocalizationTester();

		void SetInputClouds(const std::string& sceneCloudFilePath, const std::string& modelCloudFilePath, const std::string& groundTruthPoseFilePath);
		void SetGuessModelPoseInScene(const std::string& guessPoseFilePath);

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
		std::string configurationFile, extractorConfigurationFile;
		std::string sceneCloudFilePath, modelCloudFilePath, groundTruthPoseFilePath;
		std::string guessPoseFilePath;
		CDFF::DFN::Registration3DInterface* dfn;
		CDFF::DFN::FeaturesExtraction3DInterface* extractor;

		PointCloudWrapper::PointCloudConstPtr inputSceneCloud;
		PointCloudWrapper::PointCloudConstPtr inputModelCloud;
		PoseWrapper::Pose3DConstPtr inputTruthModelPoseInScene;
		PoseWrapper::Pose3DConstPtr inputGuessModelPoseInScene;
		bool outputMatcherSuccess;

		Converters::PclPointCloudToPointCloudConverter pointCloudConverter;
		bool inputsWereLoaded;
		bool groundTruthWasLoaded;
		bool guessPoseWasLoaded;

		void LoadPointClouds();
		void LoadGroudTruthPose();
		void LoadGuessPose();
		void ConfigureDfns();

		float ComputeLocationError();
		float ComputeOrientationError(float modelSize);
		float ComputeModelSize();
		void ConvertFeaturesVectorToPointCloud(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& vector, PointCloudWrapper::PointCloud& cloud);
	};

#endif

/* CorrectLocalizationTester.hpp */
/** @} */
