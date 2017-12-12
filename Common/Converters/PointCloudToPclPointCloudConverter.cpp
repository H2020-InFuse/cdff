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

using namespace CppTypes;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
 pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudToPclPointCloudConverter::Convert(const PointCloud::ConstPtr& pointCloud)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
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
