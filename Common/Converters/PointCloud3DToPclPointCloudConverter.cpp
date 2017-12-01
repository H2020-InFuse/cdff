/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloud3DToPclPointCloudConverter.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of PointCloud3DToPclPointCloudConverter.
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */

#include "PointCloud3DToPclPointCloudConverter.hpp"
#include <Errors/Assert.hpp>
#include <stdio.h>
#include <math.h>


namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
 pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud3DToPclPointCloudConverter::Convert(const PointCloud3D* pointCloud)
	{
	ASSERT(pointCloud->point_cloud_mode == PointCloud3DMode_base_mode, "PointCloud3DToPclPointCloudConverter: point cloud type not supported yet");
	PRINT_TO_LOG("size", pointCloud->size);
	PRINT_TO_LOG("count", pointCloud->data.list.count);	
	ASSERT(pointCloud->size * 3 == pointCloud->data.list.count, "PointCloud3DToPclPointCloudConverter: input point cloud data is invalid");

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>() );
	for(int pointIndex = 0; pointIndex < pointCloud->size; pointIndex++)
		{
		pcl::PointXYZ point;
		point.x = *(pointCloud->data.list.array[ pointIndex + 0 ]);
		point.y = *(pointCloud->data.list.array[ pointIndex + 1 ]);
		point.z = *(pointCloud->data.list.array[ pointIndex + 2 ]); 	
		pclPointCloud->points.push_back(point);	
		}	

	return pclPointCloud;
	}

}

/** @} */
