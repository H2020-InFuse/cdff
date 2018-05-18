/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file OutliersDetector.hpp
 * @date 14/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class is used to identify outliers in a point cloud.
 *  
 *
 * @{
 */

#ifndef OUTLIERS_DETECTOR_HPP
#define OUTLIERS_DETECTOR_HPP

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
    class OutliersDetector
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		/* @brief, the constructor takes as input the paths to the input cloud, and the path to the output file which should have xml extension. 
		*/
        	OutliersDetector(std::string inputCloudFilePath, std::string outliersFilePath);
        	~OutliersDetector();

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
		static const std::vector<cv::Scalar> COLORS_LIST;
		float visualizationCircleSize;
		
		pcl::PointXYZ visualizationCenter;
		float visualizationRadius;
		bool detectorIsActive;
		bool cloudChangedSinceLastVisualization;
		bool focusChangedSinceLastVisualization;
		std::mutex inputMutex;

		pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliersCloud;
		pcl::KdTreeFLANN<pcl::PointXYZ> originalCloudSearchTree;
		std::vector<int32_t> outliersVector;
		std::vector<int32_t> outliersBox;
		bool outliersBoxActive;

		std::string inputCloudFilePath;
		std::string outliersFilePath;

		std::vector<int32_t> visualizedIndicesList;

		pcl::visualization::PCLVisualizer::Ptr visualizer;
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>::Ptr pointCloudColorHandler;

		void LoadCloud();
		static void PointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* userdata);
		void PointPickingCallback(const pcl::visualization::PointPickingEvent& event);
		static void KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event, void* userdata);
		void KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event);

		void PrepareCloudToVisualize();
		void DrawOutliers();

		void SaveOutliers();
		void LoadOutliers();
		void ComputeCloudCenter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointXYZ& center);
		float ComputeCloudRadius(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const pcl::PointXYZ& center);
		float ComputeMinimumDistanceBetweenPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
		std::vector<int32_t> ComputeOutliersBoxClosure();
		void ComputeBoxBorders(float& minX, float& maxX, float& minY, float& maxY, float& minZ, float& maxZ);
    };

}
#endif
/* OutliersDetector.hpp */
/** @} */
