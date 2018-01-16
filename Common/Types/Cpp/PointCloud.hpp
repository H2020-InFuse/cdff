/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloud.hpp
 * @date 12/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup PointCloudWrapper
 * 
 * Namespace wrapper for the PointCloud type
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
namespace CTypes {
#include <Pointcloud.h>
}
#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>



#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

namespace PointCloudWrapper 
{
/* --------------------------------------------------------------------------
 *
 * Types definition
 *
 * --------------------------------------------------------------------------
 */
typedef CTypes::Pointcloud_colors ColorsList;
typedef CTypes::Pointcloud_points PointsList;
typedef CTypes::Pointcloud PointCloud;


/* --------------------------------------------------------------------------
 *
 * Constants definition
 *
 * --------------------------------------------------------------------------
 */
const int MAX_CLOUD_SIZE = static_cast<int>(CTypes::maxPointcloudSize);


/* --------------------------------------------------------------------------
 *
 * Shared Pointers definition
 *
 * --------------------------------------------------------------------------
 */
typedef std::shared_ptr<PointCloud> PointCloudPtr;
typedef std::shared_ptr<const PointCloud> PointCloudConstPtr;


/* --------------------------------------------------------------------------
 *
 * Functions definition
 *
 * --------------------------------------------------------------------------
 */
void Copy(const PointCloud& source, PointCloud& destination);

void AddPoint(PointCloud& pointCloud, BaseTypesWrapper::T_Double x, BaseTypesWrapper::T_Double y, BaseTypesWrapper::T_Double z);
void ClearPoints(PointCloud& pointCloud);
int GetNumberOfPoints(const PointCloud& pointCloud);
BaseTypesWrapper::T_Double GetXCoordinate(const PointCloud& pointCloud, int pointIndex);
BaseTypesWrapper::T_Double GetYCoordinate(const PointCloud& pointCloud, int pointIndex);
BaseTypesWrapper::T_Double GetZCoordinate(const PointCloud& pointCloud, int pointIndex);

}
#endif

/* PointCloud.hpp */
/** @} */
