/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PointCloudMap.hpp
 * @date 18/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * @brief This class reconstruct a 3d point cloud map by assembling smaller point clouds.
 *  
 * The map offer methods for adding more point cloud at give poses, for storing features and descriptor of point cloud for future matchings and for extracting a revelant part of the map centered at
 * a given position.
 *
 * @{
 */

#ifndef POINT_CLOUD_MAP_HPP
#define POINT_CLOUD_MAP_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <BaseTypes.hpp>
#include <VisualPointFeatureVector3D.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <stdlib.h>
#include <memory>

namespace dfpc_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class PointCloudMap
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		PointCloudMap();
		~PointCloudMap();

		/*
		* @brief Adds a point cloud at a given position.
		*
		* @param pointCloud, the point cloud to add;
		* @param pointCloudFeaturesVector, the vector of relevant features extracted from the point cloud
		* @param cloudPoseInMap, the pose of the point cloud with respect to the origin of the map
		*
		*/
		void AddPointCloud(PointCloudWrapper::PointCloudConstPtr pointCloudInput, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr pointCloudFeaturesVector,
						PoseWrapper::Pose3DConstPtr cloudPoseInMap);

		/*
		* @brief Retrieves a point cloud given by all the mapped points which are within a given radius from a center
		*
		* @param origin, the reference center for the retrivial
		* @param radius, the reference distance from the center, if radius is negative all points are selected.
		*/
		PointCloudWrapper::PointCloudConstPtr GetScenePointCloud(PoseWrapper::Pose3DConstPtr origin,  float radius);

		/*
		* @brief Retrieves the feature points located within a given radius from a center
		*
		* @param origin, the reference center for the retrivial
		* @param radius, the reference distance from the center, if radius is negative all points are selected.
		*/
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr GetSceneFeaturesVector(PoseWrapper::Pose3DConstPtr origin,  float radius);

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
		static const unsigned MAX_DESCRIPTOR_LENGTH = 352;
		static const float RESOLUTION;
		typedef float Descriptor[MAX_DESCRIPTOR_LENGTH];

		struct FeaturePoint
			{
			uint64_t pointCloudIndex;
			Descriptor descriptor;
			};

		unsigned descriptorLength;
		std::vector<FeaturePoint> featuresList;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
	
		float PointDistance(pcl::PointXYZ p, pcl::PointXYZ q);
		pcl::PointXYZ TransformPoint(pcl::PointXYZ point, PoseWrapper::Pose3DConstPtr cloudPoseInMap);
		bool AddPointToMap(pcl::PointXYZ point);
		bool IndexNotCotainedInVector(uint64_t index, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr pointCloudFeaturesVector);
    };
}
#endif
/* PointCloudMap.hpp */
/** @} */
