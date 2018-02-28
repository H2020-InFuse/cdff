/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PclVisualizer.hpp
 * @date 28/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Visualizers
 * 
 *  The OpencvVisualizer contains method for the visualization of point clouds and other PCL data structures
 * 
 * @{
 */

#ifndef PCL_VISUALIZER_HPP
#define PCL_VISUALIZER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "OpencvVisualizer.hpp"
#include<pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <stdlib.h>
#include <vector>

#include <PointCloud.hpp>
#include <VisualPointFeatureVector3D.hpp>

namespace Visualizers
{
/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PclVisualizer
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		static void ShowPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud);
		static void ShowPointClouds(std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > pointCloudsList);
		static void ShowPointCloud(PointCloudWrapper::PointCloudConstPtr pointCloud);
		static void ShowVisualFeatures(PointCloudWrapper::PointCloudConstPtr pointCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr featuresVector);

		static void Enable();
		static void Disable();
	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:
		PclVisualizer();

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		struct Color
			{
			unsigned r;
			unsigned g;
			unsigned b;
			};

		static const std::string WINDOW_NAME;
		static const unsigned MAX_POINT_CLOUDS = 10;
		static const Color COLORS_LIST[MAX_POINT_CLOUDS];
		static bool enabled;
	};

}

/* --------------------------------------------------------------------------
 *
 * Macros definition
 *
 * --------------------------------------------------------------------------
 */
	#ifndef TESTING
		#define DEBUG_SHOW_POINT_CLOUD(pointCloud)
		#define DEBUG_SHOW_POINT_CLOUDS(pointCloudsList)
		#define DEBUG_SHOW_3D_VISUAL_FEATURES(pointCloud, featuresVector)
	#else
		#define DEBUG_SHOW_POINT_CLOUD(pointCloud) Visualizers::PclVisualizer::ShowPointCloud(pointCloud)
		#define DEBUG_SHOW_POINT_CLOUDS(pointCloudsList) Visualizers::PclVisualizer::ShowPointClouds(pointCloudsList)
		#define DEBUG_SHOW_3D_VISUAL_FEATURES(pointCloud, featuresVector) Visualizers::PclVisualizer::ShowVisualFeatures(pointCloud, featuresVector)
	#endif

#endif
/* PclVisualizer.hpp */
/** @} */

