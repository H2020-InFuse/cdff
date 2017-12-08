/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudToPclPointCloudConverter.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of PointCloudToPclPointCloudConverter.
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

#include "PointCloudToPclPointCloudConverter.hpp"
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
 pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudToPclPointCloudConverter::Convert(CppTypes::PointCloud::ConstPtr pointCloud)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>() );
	for(int pointIndex = 0; pointIndex < pointCloud->GetNumberOfPoints(); pointIndex++)
		{
		pcl::PointXYZ point;
		point.x = pointCloud->GetXCoordinate(pointIndex);
		point.y = pointCloud->GetYCoordinate(pointIndex);
		point.z = pointCloud->GetZCoordinate(pointIndex); 	
		pclPointCloud->points.push_back(point);	
		}	

	return pclPointCloud;
	}

}

/** @} */
