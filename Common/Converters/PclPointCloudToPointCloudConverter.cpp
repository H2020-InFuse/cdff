/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PclPointCloudToPointCloudConverter.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of PclPointCloudToPointCloudConverter class.
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

#include "PclPointCloudToPointCloudConverter.hpp"
#include <Errors/Assert.hpp>
#include <stdio.h>
#include <math.h>

namespace Converters {

using namespace PointCloudWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
PointCloudConstPtr PclPointCloudToPointCloudConverter::Convert(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointCloud)
	{
	PointCloudPtr asnPointCloud = new PointCloud();
	for(unsigned pointIndex = 0; pointIndex < pointCloud->points.size(); pointIndex++)
		{
		AddPoint(*asnPointCloud, pointCloud->points.at(pointIndex).x, pointCloud->points.at(pointIndex).y, pointCloud->points.at(pointIndex).z);
		}
	return asnPointCloud;
	}

PointCloudSharedConstPtr PclPointCloudToPointCloudConverter::ConvertShared(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointCloud)
	{
	PointCloudConstPtr conversion = Convert(pointCloud);
	PointCloudSharedConstPtr sharedConversion(conversion);
	return sharedConversion;
	}

}

/** @} */
