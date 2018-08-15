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

namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{

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
		* @brief Retrieves a point cloud given by all the mapped points which are within a given radius from a center, the output points coordinate are relative to the scene origin.
		*
		* @param origin, the reference center for the retrivial
		* @param radius, the reference distance from the center, if radius is negative all points are selected.
		* @output, the scene point cloud in the coordinate system relative to the very first camera pose.
		*/
		PointCloudWrapper::PointCloudConstPtr GetScenePointCloud(PoseWrapper::Pose3DConstPtr origin,  float radius);

		/*
		* @brief Retrieves a point cloud given by all the mapped points which are within a given radius from a center, the output points coordinates are relative to the origin input.
		*
		* @param origin, the reference center for the retrivial, and centre of the coordinate system for the output point cloud.
		* @param radius, the reference distance from the center, if radius is negative all points are selected.
		* @output, the scene point cloud in the coordinate system relative to origin.
		*/
		PointCloudWrapper::PointCloudConstPtr GetScenePointCloudInOrigin(PoseWrapper::Pose3DConstPtr origin,  float radius);

		/*
		* @brief Retrieves the feature points located within a given radius from a center
		*
		* @param origin, the reference center for the retrivial
		* @param radius, the reference distance from the center, if radius is negative all points are selected.
		*/
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr GetSceneFeaturesVector(PoseWrapper::Pose3DConstPtr origin,  float radius);


		/*
		* @brief Set the resolution of the point cloud
		*
		* @param resolution, the resolution of the point cloud
		*
		*/
		void SetResolution(float resolution);		

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
		static const float DEFAULT_RESOLUTION;
		typedef float Descriptor[MAX_DESCRIPTOR_LENGTH];
		typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;

		struct FeaturePoint
			{
			pcl::PointXYZ point;
			Descriptor descriptor;
			};

		float resolution;
		unsigned descriptorLength;
		std::vector<FeaturePoint> featuresList;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
	
		void AddPointCloud(PointCloudWrapper::PointCloudConstPtr pointCloudInput, const AffineTransform& affineTransform);
		void AddFeatureCloud(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr pointCloudFeaturesVector, const AffineTransform& affineTransform);

		AffineTransform ConvertCloudPoseToInversionTransform(PoseWrapper::Pose3DConstPtr cloudPoseInMap);
		pcl::PointXYZ TransformPoint(const pcl::PointXYZ& point, const AffineTransform& affineTransform);
		float PointDistance(const pcl::PointXYZ& p, const pcl::PointXYZ& q);

		bool NoCloseFeature(const pcl::PointXYZ& point);
    };
}
}
}
#endif
/* PointCloudMap.hpp */
/** @} */
