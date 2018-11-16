/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file CloudsOverlapper.hpp
 * @date 13/11/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class is used to move the model around.
 *  
 *
 * @{
 */

#ifndef CLOUDS_OVERLAPPER_HPP
#define CLOUDS_OVERLAPPER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vtkRenderWindowInteractor.h>

#include <mutex>
#include <Types/CPP/Pose.hpp>

namespace DataGenerators {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class CloudsOverlapper
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		/* @brief, the constructor takes as input the paths to the input scene and model clouds 
		*/
        	CloudsOverlapper(std::string inputSceneCloudFilePath, std::string inputModelCloudFilePath);
        	~CloudsOverlapper();

		/* @brief, this method is called to start the window engine. 
		*/
		void Run();
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
		static const std::vector<cv::Scalar> COLORS_LIST;
		
		const float visualizationCircleRadius;
		bool matcherIsActive;
		bool cloudsChangedSinceLastVisualization;
		std::mutex inputMutex;

		pcl::PointCloud<pcl::PointXYZ>::Ptr originalSceneCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr originalModelCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformedModelCloud;
		PoseWrapper::Pose3D modelPose;
		float translationResolution;
		float rotationResolution;

		std::string inputSceneCloudFilePath, inputModelCloudFilePath;

		pcl::visualization::PCLVisualizer::Ptr visualizer;
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sceneCloudColor;
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> modelCloudColor;

		static const int NumberOfPoints = 4;
		bool pointsInSceneSelected[NumberOfPoints], pointsInModelSelected[NumberOfPoints];
		pcl::PointXYZ pointsInScene[NumberOfPoints], pointsInModel[NumberOfPoints];
		bool allPointsSelected, modelPoseComputed;

		void LoadClouds();
		static void PointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* userdata);
		void PointPickingCallback(const pcl::visualization::PointPickingEvent& event);
		static void KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event, void* userdata);
		void KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event);

		void VisualizePointClouds();
		void RotateModel(float rollChange, float pitchChange, float yawChange);
		pcl::PointXYZ TransformPoint(const pcl::PointXYZ& point, const AffineTransform& affineTransform);
		void ComputeModelPose();
    };

}
#endif
/* CloudsOverlapper.hpp */
/** @} */
