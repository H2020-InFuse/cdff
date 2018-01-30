/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudToPclNormalsCloudConverter.cpp
 * @date 24/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of PointCloudToPclNormalsCloudConverter.
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

#include "PointCloudToPclNormalsCloudConverter.hpp"
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
 pcl::PointCloud<pcl::Normal>::ConstPtr PointCloudToPclNormalsCloudConverter::Convert(const  PointCloudConstPtr& pointCloud)
	{
	pcl::PointCloud<pcl::Normal>::Ptr pclNormalsCloud = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*pointCloud); pointIndex++)
		{
		pcl::Normal point;
		point.normal_x = GetXCoordinate(*pointCloud, pointIndex);
		point.normal_y = GetYCoordinate(*pointCloud, pointIndex);
		point.normal_z = GetZCoordinate(*pointCloud, pointIndex); 	
		pclNormalsCloud->points.push_back(point);	
		}	

	return pclNormalsCloud;
	}

pcl::PointCloud<pcl::Normal>::ConstPtr PointCloudToPclNormalsCloudConverter::ConvertShared(const PointCloudSharedConstPtr& pointCloud)
	{
	return Convert( pointCloud.get() );
	}

}

/** @} */
