/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloud.cpp
 * @date 08/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CppTypes
 * 
 * Implementation of PointCloud class.
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

namespace CppTypes
{


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

PointCloud::PointCloud()
	{
	isColored = false;
	ClearPoints();
	}

PointCloud::~PointCloud()
	{

	}

void PointCloud::AddPoint(T_Double x, T_Double y, T_Double z)
	{
	ASSERT_ON_TEST(pointCloud.points.nCount < MAX_CLOUD_SIZE, "Point Cloud maximum capacity has been reached");
	int currentIndex = pointCloud.points.nCount;
	pointCloud.points.arr[currentIndex].arr[0] = x;
	pointCloud.points.arr[currentIndex].arr[1] = y;
	pointCloud.points.arr[currentIndex].arr[2] = z;
	pointCloud.points.nCount++;
	}

void PointCloud::ClearPoints()
	{
	pointCloud.points.nCount = 0;
	pointCloud.colors.nCount = 0;
	isColored = false;
	}

int PointCloud::GetNumberOfPoints() const
	{
	return pointCloud.points.nCount;
	}

T_Double PointCloud::GetXCoordinate(int pointIndex) const
	{
	ASSERT_ON_TEST(pointIndex < pointCloud.points.nCount, "A missing point was requested from a features vector 2D");
	return pointCloud.points.arr[pointIndex].arr[0];
	}

T_Double PointCloud::GetYCoordinate(int pointIndex) const
	{
	ASSERT_ON_TEST(pointIndex < pointCloud.points.nCount, "A missing point was requested from a features vector 2D");
	return pointCloud.points.arr[pointIndex].arr[1];
	}

T_Double PointCloud::GetZCoordinate(int pointIndex) const
	{
	ASSERT_ON_TEST(pointIndex < pointCloud.points.nCount, "A missing point was requested from a features vector 2D");
	return pointCloud.points.arr[pointIndex].arr[2];
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const int PointCloud::MAX_CLOUD_SIZE = static_cast<int>(CTypes::maxPointcloudSize);


}

/** @} */
