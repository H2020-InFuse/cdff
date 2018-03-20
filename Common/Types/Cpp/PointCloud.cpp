/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloud.cpp
 * @date 12/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup PointCloudWrapper
 * 
 * Implementation of PointCloud wrapper functions.
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
#include "PointCloud.hpp"
#include <Errors/Assert.hpp>

using namespace BaseTypesWrapper;

namespace PointCloudWrapper
{


/* --------------------------------------------------------------------------
 *
 * Functions
 *
 * --------------------------------------------------------------------------
 */
void Copy(const PointCloud& source, PointCloud& destination)
	{
	ClearPoints(destination);
	for(int pointIndex; pointIndex < GetNumberOfPoints(source); pointIndex++)
		{
		AddPoint(destination, GetXCoordinate(source, pointIndex), GetYCoordinate(source, pointIndex), GetZCoordinate(source, pointIndex) );
		}
	}

PointCloudPtr NewPointCloud()
	{
	PointCloudPtr pointCloud = new PointCloud();
	Initialize(*pointCloud);
	return pointCloud;
	}

PointCloudSharedPtr NewSharedPointCloud()
	{
	PointCloudSharedPtr sharedPointCloud = std::make_shared<PointCloud>();
	Initialize(*sharedPointCloud);
	return sharedPointCloud;
	}

void Initialize(PointCloud& pointCloud)
	{
	ClearPoints(pointCloud);
	}

void AddPoint(PointCloud& pointCloud, T_Double x, T_Double y, T_Double z)
	{
	ASSERT_ON_TEST(pointCloud.points.nCount < MAX_CLOUD_SIZE, "Point Cloud maximum capacity has been reached");
	int currentIndex = pointCloud.points.nCount;
	pointCloud.points.arr[currentIndex].arr[0] = x;
	pointCloud.points.arr[currentIndex].arr[1] = y;
	pointCloud.points.arr[currentIndex].arr[2] = z;
	pointCloud.points.nCount++;
	}

void ClearPoints(PointCloud& pointCloud)
	{
	pointCloud.points.nCount = 0;
	pointCloud.colors.nCount = 0;
	}

int GetNumberOfPoints(const PointCloud& pointCloud)
	{
	return pointCloud.points.nCount;
	}

T_Double GetXCoordinate(const PointCloud& pointCloud, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < pointCloud.points.nCount, "A missing point was requested from a features vector 2D");
	return pointCloud.points.arr[pointIndex].arr[0];
	}

T_Double GetYCoordinate(const PointCloud& pointCloud, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < pointCloud.points.nCount, "A missing point was requested from a features vector 2D");
	return pointCloud.points.arr[pointIndex].arr[1];
	}

T_Double GetZCoordinate(const PointCloud& pointCloud, int pointIndex)
	{
	ASSERT_ON_TEST(pointIndex < pointCloud.points.nCount, "A missing point was requested from a features vector 2D");
	return pointCloud.points.arr[pointIndex].arr[2];
	}

}

/** @} */
