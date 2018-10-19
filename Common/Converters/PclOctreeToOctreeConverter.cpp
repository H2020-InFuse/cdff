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
asn1SccOctree * PclOctreeToOctreeConverter::Convert(const pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& tree)
{
    asn1SccOctree * output_octree = new asn1SccOctree();
    output_octree->pointCloud = *Converters::PclPointCloudToPointCloudConverter().Convert(tree.getInputCloud());
    output_octree->resolution = tree.getResolution();
    return output_octree;
}

}

/** @} */