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

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
CppTypes::PointCloud::ConstPtr PclPointCloudToPointCloudConverter::Convert(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud)
	{
	CppTypes::PointCloud::Ptr asnPointCloud = CppTypes::PointCloud::Ptr( new CppTypes::PointCloud() );
	
	for(unsigned pointIndex = 0; pointIndex < pointCloud->points.size(); pointIndex++)
		{
		asnPointCloud->AddPoint(pointCloud->points.at(pointIndex).x, pointCloud->points.at(pointIndex).y, pointCloud->points.at(pointIndex).z);
		}

	return asnPointCloud;
	}

}

/** @} */
