/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PointCloud3DToPclPointCloudConverter.hpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from Point Cloud 3D Type to PCL Point Cloud.
 *  
 *
 * @{
 */

#ifndef POINT_CLOUD_3D_TO_PCL_POINT_CLOUD_CONVERTER_HPP
#define POINT_CLOUD_3D_TO_PCL_POINT_CLOUD_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloud3D.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PointCloud3DToPclPointCloudConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual  pcl::PointCloud<pcl::PointXYZ>::Ptr Convert(const PointCloud3D* pointCloud);

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

/* PointCloud3DToPclPointCloudConverter.hpp */
/** @} */
