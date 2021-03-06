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

#include <Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Converters/PclNormalsCloudToPointCloudConverter.hpp>
#include <Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>

#include <Errors/Assert.hpp>
#include <PerformanceTests/DFNsIntegration/PerformanceTestInterface.hpp>
#include <PerformanceTests/Aggregator.hpp>
#include <Types/CPP/Pose.hpp>

#include <Eigen/Geometry>

class DetectionDescriptionMatching3DTestInterface : public PerformanceTestInterface
	{
	public:
		struct DFNsSet
			{
			CDFF::DFN::FeaturesExtraction3DInterface* extractor;
			CDFF::DFN::FeaturesDescription3DInterface* descriptor;
			CDFF::DFN::FeaturesMatching3DInterface* matcher;
			};

		DetectionDescriptionMatching3DTestInterface(const std::string& folderPath, const std::vector<std::string>& baseConfigurationFileNamesList, 
			const std::string& performanceMeasuresFileName, DFNsSet dfnsSet);
		~DetectionDescriptionMatching3DTestInterface();

		void SetInputCloud(const std::string& inputCloudFile, float voxelGridFilterSize);
		void SetModelsCloud(std::string groundTruthTransformFilePath, std::vector<std::string> modelsCloudFilesList);
		void SetGroundTruth(float positionX, float positionY, float positionZ, float orientationX, float orientationY, float orientationZ, float orientationW);
	protected:

	private:
		pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud;
		PoseWrapper::Pose3D groundTruthPose;

		Converters::PclPointCloudToPointCloudConverter pointCloudConverter;

		PointCloudWrapper::PointCloudConstPtr scenePointCloud;
		PointCloudWrapper::PointCloudConstPtr modelPointCloud;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr sceneFeatureVector;
		bool icpSuccess;
		PoseWrapper::Pose3DPtr modelPoseInScene;
		int numberOfSceneKeypoints;
		int numberOfModelKeypoints;

		CDFF::DFN::FeaturesExtraction3DInterface* extractor;
		CDFF::DFN::FeaturesDescription3DInterface* descriptor;
		CDFF::DFN::FeaturesMatching3DInterface* matcher;

		Aggregator groundPositionDistanceAggregator;
		Aggregator groundOrientationDistanceAggregator;

		void LoadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string cloudFile);
		void LoadSceneCloud();
		void LoadModelCloud(int long inputId);

		bool SetNextInputs() override;
		void ExecuteDfns() override;
		MeasuresMap ExtractMeasures() override;

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
