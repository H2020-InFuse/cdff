/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PclOctreeToOctreeConverter.cpp
 * @date 17/09/2018
 * @authors Irene Sanz
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of PclOctreeToOctreeConverter class.
 * 
 * 
 * @{
 */

#include "PclOctreeToOctreeConverter.hpp"
#include <Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Errors/Assert.hpp>
#include <stdio.h>

namespace Converters
{

//=====================================================================================================================
asn1SccT_Octree PclOctreeToOctreeConverter::Convert(const pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& tree)
{
    asn1SccT_Octree output_octree;
    output_octree.point_cloud = Converters::PclPointCloudToPointCloudConverter().Convert(tree.getInputCloud());
    output_octree.resolution = tree.getResolution();
    return output_octree;
}

}

/** @} */