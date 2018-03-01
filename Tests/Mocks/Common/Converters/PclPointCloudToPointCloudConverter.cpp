/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PclPointCloudToPointCloudConverter.cpp
 * @date 27/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of PclPointCloudToPointCloudConverter.
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

#include "PclPointCloudToPointCloudConverter.hpp"
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

PclPointCloudToPointCloudConverter::~PclPointCloudToPointCloudConverter()
	{

	}

PointCloudConstPtr PclPointCloudToPointCloudConverter::Convert(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointCloud)
	MOCK_METHOD(Converters::PclPointCloudToPointCloudConverter, Convert, PointCloudConstPtr, (pointCloud) )

}
/** @} */
