/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PclNormalsCloudToPointCloudConverter.hpp
 * @date 24/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from Pcl Normals Cloud to ESROCOS PointCloud.
 *  
 *
 * @{
 */

#ifndef PCL_NORMALS_CLOUD_TO_POINT_CLOUD_CONVERTER_HPP
#define PCL_NORMALS_CLOUD_TO_POINT_CLOUD_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Types/CPP/PointCloud.hpp>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PclNormalsCloudToPointCloudConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual PointCloudWrapper::PointCloudConstPtr Convert(const pcl::PointCloud<pcl::Normal>::ConstPtr& pointCloud);
		PointCloudWrapper::PointCloudSharedConstPtr ConvertShared(const pcl::PointCloud<pcl::Normal>::ConstPtr& pointCloud);

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

/* PclNormalsCloudToPointCloudConverter.hpp */
/** @} */
