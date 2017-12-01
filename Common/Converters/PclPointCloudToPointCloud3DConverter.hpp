/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PclPointCloudToPointCloud3DConverter.hpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from Pcl Point Cloud to InFuse PointCloud3D.
 *  
 *
 * @{
 */

#ifndef PCL_POINT_CLOUD_TO_POINT_CLOUD_3D_CONVERTER_HPP
#define PCL_POINT_CLOUD_TO_POINT_CLOUD_3D_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <PointCloud3D.h>

namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PclPointCloudToPointCloud3DConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual PointCloud3D* Convert(const  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

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

/* PclPointCloudToPointCloud3DConverter.hpp */
/** @} */
