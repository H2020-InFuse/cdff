/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup PointCloudWrapper
 *
 * Wrapper for ASN.1 PointCloud type
 *
 * @{
 */

#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

#include <Pointcloud.h>

#include "BaseTypes.hpp"
#include <stdlib.h>
#include <memory>
#include <vector>

namespace PointCloudWrapper
{

// Types

typedef asn1SccPointCloud_Data_colors ColorsList;
typedef asn1SccPointCloud_Data_points PointsList;
typedef asn1SccPointcloud PointCloud;

// Global constant variables

const int MAX_CLOUD_SIZE = static_cast<int>(maxPointcloudSize);

// Pointer types

typedef PointCloud* PointCloudPtr;
typedef PointCloud const* PointCloudConstPtr;
typedef std::shared_ptr<PointCloud> PointCloudSharedPtr;
typedef std::shared_ptr<const PointCloud> PointCloudSharedConstPtr;

// Functions

void Copy(const PointCloud& source, PointCloud& destination);
PointCloudPtr NewPointCloud();
PointCloudSharedPtr NewSharedPointCloud();
void Initialize(PointCloud& pointCloud);

void AddPoint(PointCloud& pointCloud, BaseTypesWrapper::T_Double x, BaseTypesWrapper::T_Double y, BaseTypesWrapper::T_Double z);
void ClearPoints(PointCloud& pointCloud);
int GetNumberOfPoints(const PointCloud& pointCloud);

BaseTypesWrapper::T_Double GetXCoordinate(const PointCloud& pointCloud, int pointIndex);
BaseTypesWrapper::T_Double GetYCoordinate(const PointCloud& pointCloud, int pointIndex);
BaseTypesWrapper::T_Double GetZCoordinate(const PointCloud& pointCloud, int pointIndex);
void RemovePoints(PointCloud& pointCloud, std::vector<BaseTypesWrapper::T_UInt32> pointIndexOrderedList);

}

#endif // POINT_CLOUD_HPP

/** @} */
