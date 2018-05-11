/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SelectionTester.hpp
 * @date 11/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class will test requirement 4.1.1.8 of deliverable 5.5.
 * 90% of manually defined matches on manually defined features should be detected correctly.  Manually defined matches are considered to be matches between features made by a human 
 * on close inspection of a set of features on a pair of images.  A human will inspect each set of matches and identify those that are incorrect.
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
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>
#include <Errors/Assert.hpp>

#include <VisualPointFeatureVector3D.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <BaseTypes.hpp>
#include <CorrespondenceMap3D.hpp>
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

		void SetDfns(dfn_ci::FeaturesDescription3DInterface* descriptor, dfn_ci::FeaturesMatching3DInterface* matcher);
		void SetConfigurationFilePaths(std::string featuresDescriptorConfigurationFilePath, std::string featuresMatcherConfigurationFilePath);
		void SetInputFilesPaths(std::string sourceCloudFilePath, std::string sinkCloudFilePath, std::string correspondencesFilePath);
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
		typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;

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

		std::string featuresDescriptorConfigurationFilePath;
		std::string featuresMatcherConfigurationFilePath;
		std::string sourceCloudFilePath;
		std::string sinkCloudFilePath;
		std::string correspondencesFilePath;

		PointCloudWrapper::PointCloudConstPtr inputSourceCloud, inputSinkCloud;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputSourceKeypointsVector, inputSinkKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sourceFeaturesVector, sinkFeaturesVector;
		CorrespondenceMap3DWrapper::CorrespondenceMap3DConstPtr outputCorrespondenceMap;
		CorrespondenceMap3DWrapper::CorrespondenceMap3DConstPtr referenceCorrespondenceMap;
		PoseWrapper::Pose3DConstPtr sourcePoseInSink;
		bool matcherSuccess;

		Converters::PclPointCloudToPointCloudConverter pointCloudConverter;
		dfn_ci::FeaturesDescription3DInterface* descriptor;
		dfn_ci::FeaturesMatching3DInterface* matcher;

		bool dfnsWereLoaded;
		bool inputCloudsWereLoaded;
		bool inputKeypointsWereLoaded;
		bool precisionReferenceWasLoaded;

		void SetUpMocksAndStubs();
		void LoadInputCloud(const std::string& cloudFilePath, PointCloudWrapper::PointCloudConstPtr& cloud);
		void LoadReferenceCorrespondenceMap();
		void ConfigureDfns();

		bool ValidateCorrespondences(float percentageThreshold);
		bool CorrespondencesAreTheSame(int referenceIndex, int outputIndex);

		void ComputeMatchesFromPose();
		BaseTypesWrapper::Point3D TransformPoint(const BaseTypesWrapper::Point3D& point, const AffineTransform& affineTransform);
		BaseTypesWrapper::Point3D FindClosestSinkPointTo(const BaseTypesWrapper::Point3D& sourcePoint, float& closestDistance);
	};

#endif

/* SelectionTester.hpp */
/** @} */
