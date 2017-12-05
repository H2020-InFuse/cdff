/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloud3DToPclPointCloudConverter.hpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is a mock for the converter from PointCloud3D to pcl point cloud
 * 
 * 
 * @{
 */

#ifndef MOCKS_POINT_CLOUD_3D_TO_PCL_POINT_CLOUD_CONVERTER_HPP
#define MOCKS_POINT_CLOUD_3D_TO_PCL_POINT_CLOUD_CONVERTER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <PointCloud3DToPclPointCloudConverter.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PointCloud3DToPclPointCloudConverter : public Mock, public Converters::PointCloud3DToPclPointCloudConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~PointCloud3DToPclPointCloudConverter();
		pcl::PointCloud<pcl::PointXYZ>::Ptr Convert(const PointCloud3D* pointcloud);

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
