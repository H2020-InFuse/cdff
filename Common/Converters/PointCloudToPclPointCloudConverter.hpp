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
 *  This is the class for type conversion from Point Cloud Type to PCL Point Cloud.
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
#include <PointCloud.hpp>
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
		virtual  pcl::PointCloud<pcl::PointXYZ>::ConstPtr Convert(const PointCloudWrapper::PointCloudConstPtr& pointCloud);

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
