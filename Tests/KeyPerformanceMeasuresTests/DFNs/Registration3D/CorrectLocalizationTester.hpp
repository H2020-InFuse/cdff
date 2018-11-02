/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CorrectLocalizationTester.hpp
 * @date 29/05/2018
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
#include <Errors/Assert.hpp>

#include <PointCloud.hpp>
#include <Pose.hpp>
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
		CorrectLocalizationTester(std::string configurationFile, CDFF::DFN::Registration3DInterface* dfn);
		~CorrectLocalizationTester();

		void SetInputClouds(std::string sceneCloudFilePath, std::string modelCloudFilePath, std::string groundTruthPoseFilePath);
		void SetGuessModelPoseInScene(std::string guessPoseFilePath);

		void ExecuteDfn();
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
		std::string configurationFile;
		std::string sceneCloudFilePath, modelCloudFilePath, groundTruthPoseFilePath;
		std::string guessPoseFilePath;
		CDFF::DFN::Registration3DInterface* dfn;

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
		void ConfigureDfn();

		float ComputeLocationError();
		float ComputeOrientationError(float modelSize);
		float ComputeModelSize();
	};

#endif

/* CorrectLocalizationTester.hpp */
/** @} */
