/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file CloudsMatcher.hpp
 * @date 10/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class is used to paint keypoints matches between two point clouds.
 *  
 *
 * @{
 */

#ifndef CLOUDS_MATCHER_HPP
#define CLOUDS_MATCHER_HPP

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

namespace DataGenerators {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class CloudsMatcher
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		/* @brief, the constructor takes as input the paths to the input source and sink clouds, and the path to the output file which should have xml extension. 
		*/
        	CloudsMatcher(std::string inputSourceCloudFilePath, std::string inputSinkCloudFilePath, std::string outputCorrespondencesFilePath);
        	~CloudsMatcher();

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
		enum SelectionType
			{
			NOT_SELECTED,
			SELECTED_FROM_SOURCE,
			SELECTED_FROM_SINK,
			SELECTED_FROM_BOTH
			};

		struct Correspondence
			{
			int32_t sourceIndex;
			int32_t sinkIndex;
			SelectionType selection;
			};

		static const std::vector<cv::Scalar> COLORS_LIST;
		float visualizationCircleSize;
		
		pcl::PointXYZ sourceVisualizationCenter, sinkVisualizationCenter;
		float sourceVisualizationRadius, sinkVisualizationRadius;
		bool matcherIsActive;
		bool cloudsChangedSinceLastVisualization;
		std::mutex inputMutex;

		pcl::PointCloud<pcl::PointXYZ>::Ptr originalSourceCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr originalSinkCloud;
		pcl::KdTreeFLANN<pcl::PointXYZ> originalSourceCloudSearchTree;
		pcl::KdTreeFLANN<pcl::PointXYZ> originalSinkCloudSearchTree;
		std::vector<Correspondence> correspondencesVector;

		std::string inputSourceCloudFilePath, inputSinkCloudFilePath;
		std::string outputCorrespondencesFilePath;

		std::vector<int32_t> visualizedSourceIndicesList;
		std::vector<int32_t> visualizedSinkIndicesList;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesCloud;

		pcl::visualization::PCLVisualizer::Ptr visualizer;
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>::Ptr pointCloudColorHandler;

		void LoadClouds();
		static void PointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* userdata);
		void PointPickingCallback(const pcl::visualization::PointPickingEvent& event);
		static void KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event, void* userdata);
		void KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event);

		void PrepareCloudsToVisualize();
		void DrawCorrespondencesCloud();

		void SaveCorrespondences();
		void LoadCorrespondences();
		void ComputeCloudCenter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointXYZ& center);
		float ComputeCloudRadius(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const pcl::PointXYZ& center);
		float ComputeMinimumDistanceBetweenPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    };

}
#endif
/* CloudsMatcher.hpp */
/** @} */
