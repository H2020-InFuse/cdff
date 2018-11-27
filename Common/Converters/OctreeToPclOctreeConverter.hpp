/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
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

#ifndef OCTREE_TO_PCL_OCTREE_CONVERTER_HPP
#define OCTREE_TO_PCL_OCTREE_CONVERTER_HPP


#include <Eigen/Core>
#include <Types/C/Octree.h>
#include <pcl/octree/octree_search.h>

namespace Converters
{

class OctreeToPclOctreeConverter
{
	public:
		const pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Convert(const asn1SccOctree& data);
};
}

#endif //OCTREE_TO_PCL_OCTREE_CONVERTER_HPP
/** @} */
