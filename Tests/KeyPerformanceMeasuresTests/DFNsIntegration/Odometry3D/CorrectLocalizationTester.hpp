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

#include <VisualPointFeatureVector3D.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>

#include <ConversionCache/ConversionCache.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/EigenTransformToTransform3DConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclNormalsCloudConverter.hpp>

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
		Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudWrapper::PointCloudConstPtr>* stubCloudCache;
		Mocks::PclPointCloudToPointCloudConverter* mockCloudConverter;

		Stubs::CacheHandler<PointCloudWrapper::PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>* stubInverseCloudCache;
		Mocks::PointCloudToPclPointCloudConverter* mockInverseCloudConverter;

		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr>* stubVector3dCache;
		Mocks::MatToVisualPointFeatureVector3DConverter* mockVector3dConverter;

		Stubs::CacheHandler<PointCloudWrapper::PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr >* stubNormalsCache;
		Mocks::PointCloudToPclNormalsCloudConverter* mockNormalsConverter;

		Stubs::CacheHandler<VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr, Converters::SupportTypes::PointCloudWithFeatures >* stubFeaturesCloudCache;
		Mocks::VisualPointFeatureVector3DToPclPointCloudConverter* mockFeaturesCloudConverter;

		Stubs::CacheHandler<Eigen::Matrix4f, PoseWrapper::Transform3DConstPtr>* stubTransformCache;
		Mocks::EigenTransformToTransform3DConverter* mockTransformConverter;

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

		void ExtractFeatures();
		void DescribeFeatures();
		void MatchFeatures();

		void SetUpMocksAndStubs();
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
