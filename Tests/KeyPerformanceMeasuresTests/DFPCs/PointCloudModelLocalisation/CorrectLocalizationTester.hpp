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
#include <PointCloudModelLocalisation/PointCloudModelLocalisationInterface.hpp>
#include <Errors/Assert.hpp>
#include <ctime>

#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Pose.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Converters/SupportTypes.hpp>

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

		void SetDfpc(const std::string& configurationFilePath, CDFF::DFPC::PointCloudModelLocalisationInterface* dfpc);
		void SetInputClouds(const std::string& sceneCloudFilePath, const std::string& modelCloudFilePath, const std::string& groundTruthPoseFilePath);

		void ExecuteDfpc(bool showClouds = false);
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
		typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;

		std::string dfpcConfigurationFilePath;
		std::string sceneCloudFilePath, modelCloudFilePath, groundTruthPoseFilePath;
		CDFF::DFPC::PointCloudModelLocalisationInterface* dfpc;

		PointCloudWrapper::PointCloudConstPtr inputSceneCloud;
		PointCloudWrapper::PointCloudConstPtr inputModelCloud;
		PoseWrapper::Pose3DConstPtr inputTruthModelPoseInScene;
		PoseWrapper::Pose3DConstPtr outputModelPoseInScene;
		bool outputMatcherSuccess;

		pcl::PointCloud<pcl::PointXYZ>::Ptr baseScenePclCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr baseModelPclCloud;

		Converters::PclPointCloudToPointCloudConverter pointCloudConverter;
		bool dfpcWasConfigured;
		bool inputsWereLoaded;
		bool groundTruthWasLoaded;
		clock_t beginTime, endTime;
		float processingTime;

		void Localize();

		void LoadPointClouds();
		void LoadGroudTruthPose();
		void ConfigureDfpc();

		float ComputeLocationError();
		float ComputeOrientationError(float modelSize);
		float ComputeModelSize();

		void ShowClouds();
		pcl::PointXYZ TransformPoint(const pcl::PointXYZ& point, const AffineTransform& affineTransform);
	};

#endif

/* CorrectLocalizationTester.hpp */
/** @} */
