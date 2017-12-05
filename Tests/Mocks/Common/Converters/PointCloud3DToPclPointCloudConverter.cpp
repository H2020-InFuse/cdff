/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloud3DToPclPointCloudConverter.cpp
 * @date 01/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of PointCloud3DToPclPointCloudConverter.
 * 
 * 
 * @{
 */


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */

#include "PointCloud3DToPclPointCloudConverter.hpp"
#include <Errors/Assert.hpp>
#include <Mocks/MockMacro.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

PointCloud3DToPclPointCloudConverter::~PointCloud3DToPclPointCloudConverter()
	{

	}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud3DToPclPointCloudConverter::Convert(const PointCloud3D* pointcloud)
	MOCK_METHOD(Converters::PointCloud3DToPclPointCloudConverter, Convert, pcl::PointCloud<pcl::PointXYZ>::Ptr, (pointcloud) )


}
/** @} */
