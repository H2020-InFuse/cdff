/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PclNormalsCloudToPointCloudConverter.cpp
 * @date 24/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of PclNormalsCloudToPointCloudConverter class.
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

#include "PclNormalsCloudToPointCloudConverter.hpp"
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
PointCloudConstPtr PclNormalsCloudToPointCloudConverter::Convert(const pcl::PointCloud<pcl::Normal>::ConstPtr& pointCloud)
	{
	PointCloudPtr asnPointCloud = new PointCloud();
	for(unsigned pointIndex = 0; pointIndex < pointCloud->points.size(); pointIndex++)
		{
		AddPoint(*asnPointCloud, pointCloud->points.at(pointIndex).normal_x, pointCloud->points.at(pointIndex).normal_y, pointCloud->points.at(pointIndex).normal_z);
		}
	return asnPointCloud;
	}

PointCloudSharedConstPtr PclNormalsCloudToPointCloudConverter::ConvertShared(const pcl::PointCloud<pcl::Normal>::ConstPtr& pointCloud)
	{
	PointCloudConstPtr conversion = Convert(pointCloud);
	PointCloudSharedConstPtr sharedConversion(conversion);
	return sharedConversion;
	}

}

/** @} */
