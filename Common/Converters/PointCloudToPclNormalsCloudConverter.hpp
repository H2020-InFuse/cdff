/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PointCloudToPclNormalsCloudConverter.hpp
 * @date 24/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from ESROCOS Point Cloud Type to PCL Normals Cloud.
 *  
 *
 * @{
 */

#ifndef POINT_CLOUD_TO_PCL_NORMALS_CLOUD_CONVERTER_HPP
#define POINT_CLOUD_TO_PCL_NORMALS_CLOUD_CONVERTER_HPP


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
class PointCloudToPclNormalsCloudConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual pcl::PointCloud<pcl::Normal>::ConstPtr Convert(const PointCloudWrapper::PointCloudConstPtr& pointCloud);
		pcl::PointCloud<pcl::Normal>::ConstPtr ConvertShared(const PointCloudWrapper::PointCloudSharedConstPtr& pointCloud);

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

/* PointCloudToPclNormalsCloudConverter.hpp */
/** @} */
