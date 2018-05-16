/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ShapesProfiler.hpp
 * @date 15/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class is used to record measures of lines along the shape of object and lines connecting the object to the camera position.
 *  
 *
 * @{
 */

#ifndef SHAPES_PROFILER_HPP
#define SHAPES_PROFILER_HPP

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
    class ShapesProfiler
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		/* @brief, the constructor takes as input the paths to the input cloud, and the path to the output file which should have xml extension. 
		*/
        	ShapesProfiler(std::string inputCloudFilePath, std::string shapesFilePath);
        	~ShapesProfiler();

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
		enum ProfilerState
			{
			START_STATE,
			POINT_TO_CAMERA_PICKED,
			FIRST_LINE_POINT_PICKED,
			SECOND_LINE_POINT_PICKED
			};

		enum ProfilerMode
			{
			PROFILE_OBJECTS,
			PROFILE_DISTANCES_TO_CAMERA,
			INSPECTION
			};

		struct Point
			{
			int32_t index;
			float distanceToCamera;
			};

		struct Line
			{
			int32_t sourceIndex;
			int32_t sinkIndex;
			float length;
			};

		typedef std::vector<Line> Object;

		static const std::vector<cv::Scalar> COLORS_LIST;
		static pcl::PointXYZRGB nonVisualizedPoint;
		float visualizationCircleSize;
		
		pcl::PointXYZ visualizationCenter;
		float visualizationRadius;
		bool profilerIsActive;
		bool cloudChangedSinceLastVisualization;
		bool focusChangedSinceLastVisualization;
		std::mutex inputMutex;

		pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr shapesCloud;
		pcl::KdTreeFLANN<pcl::PointXYZ> originalCloudSearchTree;

		std::vector<Object> objectsList;
		std::vector<Point> pointsToCameraList;
		int activeObject;
		int totalLinesCounter;
		ProfilerMode profilerMode;
		ProfilerState profilerState;

		pcl::PointXYZ cameraCoordinates;
		bool cameraCoordinatesWereSet;

		std::string inputCloudFilePath;
		std::string shapesFilePath;

		std::vector<int32_t> visualizedIndicesList;

		pcl::visualization::PCLVisualizer::Ptr visualizer;
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>::Ptr pointCloudColorHandler;

		void LoadCloud();
		static void PointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* userdata);
		void PointPickingCallback(const pcl::visualization::PointPickingEvent& event);
		void HandleObjectsModePointPicking(int32_t originalPointIndex);
		void HandlePointsModePointPicking(int32_t originalPointIndex);
		void HandleInspectionPointPicking(int32_t originalPointIndex);
		static void KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event, void* userdata);
		void KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event);

		void PrepareCloudToVisualize();
		void DrawObjects();
		void DrawObject(int objectIndex, const cv::Scalar& color);
		pcl::PointXYZRGB& GetVisualizedPoint(int32_t originalPointIndex, bool& pointIsVisualized);
		void DrawPointsToCamera();
		void VisualizeCloud();

		void SaveObjectsAndPoints();
		bool LoadObjectsAndPoints();
		void ComputeCloudCenter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointXYZ& center);
		float ComputeCloudRadius(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const pcl::PointXYZ& center);
		float ComputeMinimumDistanceBetweenPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
		float ComputeLineLength(int32_t sourceIndex, int32_t sinkIndex);
		float ComputeDistanceToCameraCoordinates(int32_t index);
    };

}
#endif
/* ShapesProfiler.hpp */
/** @} */
