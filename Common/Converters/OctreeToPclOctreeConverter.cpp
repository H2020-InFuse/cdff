/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OctreeToPclOctreeConverter.cpp
 * @date 17/09/2018
 * @authors Irene Sanz
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of OctreeToPclOctreeConverter class.
 * 
 * 
 * @{
 */

#include "OctreeToPclOctreeConverter.hpp"
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Errors/Assert.hpp>

namespace Converters
{

//=====================================================================================================================
const pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> OctreeToPclOctreeConverter::Convert(const asn1SccOctree& data)
{
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> pcl_octree (data.resolution);
    pcl_octree.setInputCloud (Converters::PointCloudToPclPointCloudConverter().Convert(&data.pointCloud));
    pcl_octree.addPointsFromInputCloud();
    return pcl_octree;
}

}

/** @} */
