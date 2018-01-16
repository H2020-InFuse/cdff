/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudToPclPointCloudConverter.cpp
 * @date 01/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of PointCloudToPclPointCloudConverter.
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

#include "PointCloudToPclPointCloudConverter.hpp"
#include <Errors/Assert.hpp>
#include <Mocks/MockMacro.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

PointCloudToPclPointCloudConverter::~PointCloudToPclPointCloudConverter()
	{

	}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudToPclPointCloudConverter::Convert(const PointCloudWrapper::PointCloudConstPtr& pointCloud)
	MOCK_METHOD(Converters::PointCloudToPclPointCloudConverter, Convert, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, (pointCloud) )

pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudToPclPointCloudConverter::Convert(const PointCloudWrapper::PointCloud& pointCloud)
	MOCK_METHOD(Converters::PointCloudToPclPointCloudConverter, Convert, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, (pointCloud) )

void PointCloudToPclPointCloudConverter::Convert(const PointCloudWrapper::PointCloud& pointCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr& conversion)
	MOCK_VOID_METHOD(Converters::PointCloudToPclPointCloudConverter, Convert, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, (pointCloud), (conversion) )

}
/** @} */
