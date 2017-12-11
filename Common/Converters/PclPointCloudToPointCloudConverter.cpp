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

using namespace CppTypes;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
PointCloud::ConstPtr PclPointCloudToPointCloudConverter::Convert(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud)
	{
	PointCloud::Ptr asnPointCloud = PointCloud::Ptr( new PointCloud() );
	
	for(unsigned pointIndex = 0; pointIndex < pointCloud->points.size(); pointIndex++)
		{
		asnPointCloud->AddPoint(pointCloud->points.at(pointIndex).x, pointCloud->points.at(pointIndex).y, pointCloud->points.at(pointIndex).z);
		}

	return asnPointCloud;
	}

}

/** @} */
