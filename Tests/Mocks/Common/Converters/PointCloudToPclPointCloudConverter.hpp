/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudToPclPointCloudConverter.hpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is a mock for the converter from PointCloud to pcl point cloud
 * 
 * 
 * @{
 */

#ifndef MOCKS_POINT_CLOUD_TO_PCL_POINT_CLOUD_CONVERTER_HPP
#define MOCKS_POINT_CLOUD_TO_PCL_POINT_CLOUD_CONVERTER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <PointCloudToPclPointCloudConverter.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PointCloudToPclPointCloudConverter : public Mock, public Converters::PointCloudToPclPointCloudConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~PointCloudToPclPointCloudConverter();
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr Convert(const PointCloudWrapper::PointCloudConstPtr& pointcloud);
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr Convert(const PointCloudWrapper::PointCloud& pointCloud);
		void Convert(const PointCloudWrapper::PointCloud& pointCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr& conversion);

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
