/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DetectionDescriptionMatching3D.cpp
 * @date 27/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * This class is the main class for the performance test of the integration pipeline of the
 * following three DFNs: FeatureExtraction3D, FeatureDescription3D and FeatureMatching3D
 * 
 * 
 * @{
 */

#ifndef DETECTION_DESCRIPTION_MATCHING_TEST_INTERFACE_3D
#define DETECTION_DESCRIPTION_MATCHING_TEST_INTERFACE_3D

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <fstream>
#include <string>

#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>

#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Mocks/Common/Converters/PointCloudToPclNormalsCloudConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/EigenTransformToTransform3DConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <PclNormalsCloudToPointCloudConverter.hpp>
#include <VisualPointFeatureVector3DToPclPointCloudConverter.hpp>

#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNsIntegration/PerformanceTestInterface.hpp>
#include <PerformanceTests/Aggregator.hpp>

#include <Eigen/Geometry>

class DetectionDescriptionMatching3DTestInterface : public PerformanceTestInterface
	{
	public:
		struct DFNsSet
			{
			dfn_ci::FeaturesExtraction3DInterface* extractor;
			dfn_ci::FeaturesDescription3DInterface* descriptor;
			dfn_ci::FeaturesMatching3DInterface* matcher;
			};

		DetectionDescriptionMatching3DTestInterface(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, std::string performanceMeasuresFileName, DFNsSet dfnsSet);
		~DetectionDescriptionMatching3DTestInterface();

		void SetInputCloud(std::string inputCloudFile, float voxelGridFilterSize);
		void SetModelsCloud(std::string groundTruthTransformFilePath, std::vector<std::string> modelsCloudFilesList); 
		void SetGroundTruth(float positionX, float positionY, float positionZ, float orientationX, float orientationY, float orientationZ, float orientationW);
	protected:

	private:
		Stubs::CacheHandler<PointCloudWrapper::PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr >* stubNormalsCache;
		Mocks::PointCloudToPclNormalsCloudConverter* mockNormalsConverter;

		Stubs::CacheHandler<PointCloudWrapper::PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr >* stubPointCloudCache;
		Mocks::PointCloudToPclPointCloudConverter* mockPointCloudConverter;

		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr>* stubFeatures3dCache;
		Mocks::MatToVisualPointFeatureVector3DConverter* mockFeatures3dConverter;

		Stubs::CacheHandler<VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr, Converters::SupportTypes::PointCloudWithFeatures >* stubFeaturesCloudCache;
		Mocks::VisualPointFeatureVector3DToPclPointCloudConverter* mockFeaturesCloudConverter;

		Stubs::CacheHandler<Eigen::Matrix4f, PoseWrapper::Transform3DConstPtr>* stubTransformCache;
		Mocks::EigenTransformToTransform3DConverter* mockTransformConverter;

		pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud;
		PoseWrapper::Pose3D groundTruthPose;

		Converters::PclPointCloudToPointCloudConverter pointCloudConverter;

		PointCloudWrapper::PointCloudConstPtr scenePointCloud;
		PointCloudWrapper::PointCloudConstPtr modelPointCloud;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sceneKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr modelKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sceneFeaturesVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr modelFeaturesVector;
		bool icpSuccess;
		PoseWrapper::Pose3DConstPtr modelPoseInScene;

		dfn_ci::FeaturesExtraction3DInterface* extractor;
		dfn_ci::FeaturesDescription3DInterface* descriptor;
		dfn_ci::FeaturesMatching3DInterface* matcher;

		Aggregator* groundPositionDistanceAggregator;
		Aggregator* groundOrientationDistanceAggregator;

		void LoadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string cloudFile);
		void LoadSceneCloud();
		void LoadModelCloud(int long inputId);

		void SetupMocksAndStubs();
		bool SetNextInputs();
		void ExecuteDfns();
		MeasuresMap ExtractMeasures();

		int long inputId;
		std::string inputCloudFile;
		std::string groundTruthTransformFilePath;
		float voxelGridFilterSize;
		std::vector<std::string> modelsCloudFilesList;
		std::vector<PoseWrapper::Pose3D> posesList;

		void ComputeDistanceToGroundTruth(float& positionDistance, float& angleDistance);
	};

#endif

/* DetectionDescriptionMatching3D.hpp */
/** @} */
