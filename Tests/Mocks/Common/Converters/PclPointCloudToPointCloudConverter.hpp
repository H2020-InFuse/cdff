/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PclPointCloudToPointCloudConverter.hpp
 * @date 27/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is a mock for the converter from pcl point cloud to point cloud
 * 
 * 
 * @{
 */

#ifndef MOCKS_PCL_POINT_CLOUD_TO_POINT_CLOUD_CONVERTER_HPP
#define MOCKS_PCL_POINT_CLOUD_TO_POINT_CLOUD_CONVERTER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <Converters/PclPointCloudToPointCloudConverter.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PclPointCloudToPointCloudConverter : public Mock, public Converters::PclPointCloudToPointCloudConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~PclPointCloudToPointCloudConverter();
		PointCloudWrapper::PointCloudConstPtr Convert(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointCloud) override;

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
