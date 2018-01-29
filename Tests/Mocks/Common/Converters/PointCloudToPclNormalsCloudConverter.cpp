/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudToPclNormalsCloudConverter.cpp
 * @date 24/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of PointCloudToPclNormalsCloudConverter.
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

#include "PointCloudToPclNormalsCloudConverter.hpp"
#include <Errors/Assert.hpp>
#include <Mocks/MockMacro.hpp>

namespace Mocks {

using namespace PointCloudWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

PointCloudToPclNormalsCloudConverter::~PointCloudToPclNormalsCloudConverter()
	{

	}

pcl::PointCloud<pcl::Normal>::ConstPtr PointCloudToPclNormalsCloudConverter::Convert(const PointCloudConstPtr& pointCloud)
	MOCK_METHOD(Converters::PointCloudToPclNormalsCloudConverter, Convert, pcl::PointCloud<pcl::Normal>::ConstPtr, (pointCloud) )

}
/** @} */
