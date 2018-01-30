/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudToPclNormalsCloudConverter.hpp
 * @date 24/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is a mock for the converter from PointCloud to pcl normals cloud
 * 
 * 
 * @{
 */

#ifndef MOCKS_POINT_CLOUD_TO_PCL_NORMALS_CLOUD_CONVERTER_HPP
#define MOCKS_POINT_CLOUD_TO_PCL_NORMALS_CLOUD_CONVERTER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <PointCloudToPclNormalsCloudConverter.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PointCloudToPclNormalsCloudConverter : public Mock, public Converters::PointCloudToPclNormalsCloudConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~PointCloudToPclNormalsCloudConverter();
		pcl::PointCloud<pcl::Normal>::ConstPtr Convert(const PointCloudWrapper::PointCloudConstPtr& pointCloud);

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
