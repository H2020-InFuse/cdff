/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PointCloudTransformer.hpp
 * @date 23/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class is for the transformation of a base point cloud into another point cloud, with the objective of extracting a portion of the cloud, transforming it into another different coordinate
 *  system and adding some artificial noie.
 *  
 *
 * @{
 */

#ifndef POINT_CLOUD_TRANSFORMER_HPP
#define POINT_CLOUD_TRANSFORMER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <random>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

namespace DataGenerators {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class PointCloudTransformer
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
        	PointCloudTransformer();
        	~PointCloudTransformer();

		void LoadPointCloud(std::string pointCloudFilePath);
		void Resize(unsigned minIndex, unsigned maxIndex);
		void TransformCloud(float positionX, float positionY, float positionZ, float rotationX, float rotationY, float rotationZ, float rotationW);
		void TransformCamera(float positionX, float positionY, float positionZ, float rotationX, float rotationY, float rotationZ, float rotationW);
		void AddGaussianNoise(float mean, float standardDeviation);
		void RemoveOutliers();
		void SavePointCloud(std::string outputFilePath);
		void ViewPointCloud();

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

		std::default_random_engine randomEngine;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud;

		bool transformedCloudWasInitialized;

		void InitTransformedCloud();
		pcl::PointXYZ TransformPoint(const pcl::PointXYZ& point, const AffineTransform& affineTransform);
		Eigen::Quaternion<float> InvertQuaternion(Eigen::Quaternion<float> input);

    };

}
#endif
/* PointCloudTransformer.hpp */
/** @} */
