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
#include <boost/smart_ptr.hpp>


namespace Converters {

using namespace PointCloudWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
 pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudToPclPointCloudConverter::Convert(const  PointCloudConstPtr& pointCloud)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*pointCloud); pointIndex++)
		{
		pcl::PointXYZ point;
		point.x = GetXCoordinate(*pointCloud, pointIndex);
		point.y = GetYCoordinate(*pointCloud, pointIndex);
		point.z = GetZCoordinate(*pointCloud, pointIndex); 	
		pclPointCloud->points.push_back(point);	
		}	

	return pclPointCloud;
	}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudToPclPointCloudConverter::ConvertShared(const PointCloudSharedConstPtr& pointCloud)
	{
	return Convert( pointCloud.get() );
	}

}

/** @} */
