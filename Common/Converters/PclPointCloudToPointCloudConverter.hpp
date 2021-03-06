/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PclPointCloudToPointCloudConverter.hpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from Pcl Point Cloud to ESROCOS PointCloud.
 *  
 *
 * @{
 */

#ifndef PCL_POINT_CLOUD_TO_POINT_CLOUD_CONVERTER_HPP
#define PCL_POINT_CLOUD_TO_POINT_CLOUD_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Types/CPP/PointCloud.hpp>
#include <Errors/AssertOnTest.hpp>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PclPointCloudToPointCloudConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual PointCloudWrapper::PointCloudConstPtr Convert(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointCloud);
		PointCloudWrapper::PointCloudSharedConstPtr ConvertShared(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointCloud);

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

/* PclPointCloudToPointCloudConverter.hpp */
/** @} */
