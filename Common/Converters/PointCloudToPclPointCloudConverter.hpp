/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PointCloudToPclPointCloudConverter.hpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from ESROCOS Point Cloud Type to PCL Point Cloud.
 *  
 *
 * @{
 */

#ifndef POINT_CLOUD_TO_PCL_POINT_CLOUD_CONVERTER_HPP
#define POINT_CLOUD_TO_PCL_POINT_CLOUD_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Types/CPP/PointCloud.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PointCloudToPclPointCloudConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual pcl::PointCloud<pcl::PointXYZ>::ConstPtr Convert(const PointCloudWrapper::PointCloudConstPtr& pointCloud);
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr ConvertShared(const PointCloudWrapper::PointCloudSharedConstPtr& pointCloud);

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
	};

}

#endif

/* PointCloudToPclPointCloudConverter.hpp */
/** @} */
