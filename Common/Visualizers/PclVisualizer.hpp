/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup Visualizers
 * @{
 */

#ifndef PCL_VISUALIZER_HPP
#define PCL_VISUALIZER_HPP

#include "OpencvVisualizer.hpp"
#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Types/CPP/Pose.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <stdlib.h>
#include <vector>

namespace Visualizers
{

/**
 * PclVisualizer: a class for displaying pointclouds
 *                and other PCL data structures
 */
class PclVisualizer
{
	public:

		static void ShowPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud);
		static void ShowPointClouds(std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > pointCloudsList);
		static void ShowPointCloud(PointCloudWrapper::PointCloudConstPtr pointCloud);
		static void ShowVisualFeatures(PointCloudWrapper::PointCloudConstPtr pointCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr featuresVector);
		static void ShowImage(pcl::PointCloud<pcl::RGB>::ConstPtr image);
		static void ShowPoses(std::vector<PoseWrapper::Pose3D> poseList);

		static void PlacePointCloud(PointCloudWrapper::PointCloudConstPtr sceneCloud, PointCloudWrapper::PointCloudConstPtr objectCloud, PoseWrapper::Pose3DConstPtr objectPoseInScene);
		static void ShowMatches(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud2, const std::vector<int>& indexList1, 
			const std::vector<int>& indexList2);

		static void SavePointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, unsigned period = 1);
		static void SavePointCloud(PointCloudWrapper::PointCloudConstPtr pointCloud, unsigned period = 1);

		static void Enable();
		static void Disable();
		static void EnableSaving();
		static void DisableSaving();

	protected:

		PclVisualizer();

	private:

		struct Color
		{
			unsigned r, g, b;
		};

		static const std::string WINDOW_NAME;
		static const std::string SAVE_FILE_BASE_NAME;
		static const std::string SAVE_FILE_EXTENSION;
		static const unsigned MAX_POINT_CLOUDS = 10;
		static const Color COLORS_LIST[MAX_POINT_CLOUDS];

		static bool enabled;
		static bool enabledSaving;

		static pcl::PointXYZ TransformPoint(pcl::PointXYZ point, PoseWrapper::Transform3DConstPtr transform);
};

}

#ifndef TESTING
	#define DEBUG_SHOW_POINT_CLOUD(pointCloud)
	#define DEBUG_SHOW_POINT_CLOUDS(pointCloudsList)
	#define DEBUG_SHOW_3D_VISUAL_FEATURES(pointCloud, featuresVector)
	#define DEBUG_PLACE_POINT_CLOUD(scene, object, objectPose)
	#define DEBUG_SHOW_PCL_IMAGE(image)
	#define DEBUG_SAVE_POINT_CLOUD(pointCloud)
	#define DEBUG_SAVE_POINT_CLOUD_WITH_PERIOD(pointCloud, period)
	#define DEBUG_SHOW_MATCHES(cloud1, cloud2, indexList1, indexList2)
	#define DEBUG_SHOW_POSES(poseList)
#else
	#define DEBUG_SHOW_POINT_CLOUD(pointCloud) Visualizers::PclVisualizer::ShowPointCloud(pointCloud)
	#define DEBUG_SHOW_POINT_CLOUDS(pointCloudsList) Visualizers::PclVisualizer::ShowPointClouds(pointCloudsList)
	#define DEBUG_SHOW_3D_VISUAL_FEATURES(pointCloud, featuresVector) Visualizers::PclVisualizer::ShowVisualFeatures(pointCloud, featuresVector)
	#define DEBUG_PLACE_POINT_CLOUD(scene, object, objectPose) Visualizers::PclVisualizer::PlacePointCloud(scene, object, objectPose)
	#define DEBUG_SHOW_PCL_IMAGE(image) Visualizers::PclVisualizer::ShowImage(image)
	#define DEBUG_SAVE_POINT_CLOUD(pointCloud) Visualizers::PclVisualizer::SavePointCloud(pointCloud)
	#define DEBUG_SAVE_POINT_CLOUD_WITH_PERIOD(pointCloud, period) Visualizers::PclVisualizer::SavePointCloud(pointCloud, period)
	#define DEBUG_SHOW_MATCHES(cloud1, cloud2, indexList1, indexList2) Visualizers::PclVisualizer::ShowMatches(cloud1, cloud2, indexList1, indexList2);
	#define DEBUG_SHOW_POSES(poseList) Visualizers::PclVisualizer::ShowPoses(poseList);
#endif

#endif // PCL_VISUALIZER_HPP

/** @} */
