/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
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

#ifndef PCL_OCTREE_TO_OCTREE_CONVERTER_HPP
#define PCL_OCTREE_TO_OCTREE_CONVERTER_HPP


#include <Eigen/Core>
#include <Types/CPP/BaseTypes.hpp>
#include <Voxelization/Octree.hpp>

namespace Converters
{

class PclOctreeToOctreeConverter
{
	public:
	asn1SccOctree * Convert(const pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& tree);
};
}

#endif //PCL_OCTREE_TO_OCTREE_CONVERTER_HPP
/** @} */
