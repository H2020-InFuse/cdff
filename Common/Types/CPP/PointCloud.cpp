/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup PointCloudWrapper
 * @{
 */

#include "PointCloud.hpp"
#include <Errors/Assert.hpp>

namespace PointCloudWrapper
{

using namespace BaseTypesWrapper;

void Copy(const PointCloud& source, PointCloud& destination)
{
	ClearPoints(destination);
	for (int pointIndex = 0; pointIndex < GetNumberOfPoints(source); pointIndex++)
	{
		AddPoint(destination, GetXCoordinate(source, pointIndex), GetYCoordinate(source, pointIndex), GetZCoordinate(source, pointIndex));
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
	ASSERT_ON_TEST(pointCloud.data.points.nCount < MAX_CLOUD_SIZE, "Point Cloud maximum capacity has been reached");
	int currentIndex = pointCloud.data.points.nCount;
	pointCloud.data.points.arr[currentIndex].arr[0] = x;
	pointCloud.data.points.arr[currentIndex].arr[1] = y;
	pointCloud.data.points.arr[currentIndex].arr[2] = z;
	pointCloud.data.points.nCount++;
}

void AddColorToLastPoint(PointCloud& pointCloud, BaseTypesWrapper::T_Double r, BaseTypesWrapper::T_Double g, BaseTypesWrapper::T_Double b, BaseTypesWrapper::T_Double alpha)
	{
	int lastAddedIndex = pointCloud.data.points.nCount - 1;
	ASSERT(lastAddedIndex >= 0, "Point Cloud error: color could not be added, cloud is empty");

	pointCloud.data.colors.arr[lastAddedIndex].arr[0] = r;
	pointCloud.data.colors.arr[lastAddedIndex].arr[1] = g;
	pointCloud.data.colors.arr[lastAddedIndex].arr[2] = b;
	//pointCloud.data.colors.arr[lastAddedIndex].arr[3] = alpha;
	pointCloud.data.colors.nCount = lastAddedIndex + 1;
	}

void ClearPoints(PointCloud& pointCloud)
{
	pointCloud.data.points.nCount = 0;
	pointCloud.data.colors.nCount = 0;
	pointCloud.data.intensity.nCount = 0;
	for(int pointIndex = 0; pointIndex < MAX_CLOUD_SIZE; pointIndex++)
		{
		pointCloud.data.points.arr[pointIndex].nCount = 3;
		pointCloud.data.colors.arr[pointIndex].nCount = 3; // = 4;
		}
	
	pointCloud.metadata.sensorId.nCount = 0;
	pointCloud.metadata.frameId.nCount = 0;
	pointCloud.metadata.timeStamp.microseconds = 0;
	pointCloud.metadata.timeStamp.usecPerSec = 0;
	pointCloud.metadata.width = 0;
	pointCloud.metadata.height = 0;
	pointCloud.metadata.msgVersion = 0;
	pointCloud.metadata.pose_robotFrame_sensorFrame.data.translation.nCount = 3;
	pointCloud.metadata.pose_robotFrame_sensorFrame.data.orientation.nCount = 4;
	pointCloud.metadata.pose_robotFrame_sensorFrame.data.cov.nCount = 6;
	pointCloud.metadata.pose_robotFrame_sensorFrame.metadata.producerId.nCount = 0;
	pointCloud.metadata.pose_robotFrame_sensorFrame.metadata.parentFrameId.nCount = 0;
	pointCloud.metadata.pose_robotFrame_sensorFrame.metadata.childFrameId.nCount = 0;
	pointCloud.metadata.pose_robotFrame_sensorFrame.metadata.msgVersion = 0;
	pointCloud.metadata.pose_robotFrame_sensorFrame.metadata.parentTime.microseconds = 0;
	pointCloud.metadata.pose_robotFrame_sensorFrame.metadata.parentTime.usecPerSec = 0;
	pointCloud.metadata.pose_robotFrame_sensorFrame.metadata.childTime.microseconds = 0;
	pointCloud.metadata.pose_robotFrame_sensorFrame.metadata.childTime.usecPerSec = 0;

	pointCloud.metadata.pose_fixedFrame_robotFrame.data.translation.nCount = 3;
	pointCloud.metadata.pose_fixedFrame_robotFrame.data.orientation.nCount = 4;
	pointCloud.metadata.pose_fixedFrame_robotFrame.data.cov.nCount = 6;
	pointCloud.metadata.pose_fixedFrame_robotFrame.metadata.producerId.nCount = 0;
	pointCloud.metadata.pose_fixedFrame_robotFrame.metadata.parentFrameId.nCount = 0;
	pointCloud.metadata.pose_fixedFrame_robotFrame.metadata.childFrameId.nCount = 0;
	pointCloud.metadata.pose_fixedFrame_robotFrame.metadata.msgVersion = 0;
	pointCloud.metadata.pose_fixedFrame_robotFrame.metadata.parentTime.microseconds = 0;
	pointCloud.metadata.pose_fixedFrame_robotFrame.metadata.parentTime.usecPerSec = 0;
	pointCloud.metadata.pose_fixedFrame_robotFrame.metadata.childTime.microseconds = 0;
	pointCloud.metadata.pose_fixedFrame_robotFrame.metadata.childTime.usecPerSec = 0;

	for(int row = 0; row < 6; row++)
		{
		pointCloud.metadata.pose_robotFrame_sensorFrame.data.cov.arr[row].nCount = 6;
		pointCloud.metadata.pose_fixedFrame_robotFrame.data.cov.arr[row].nCount = 6;
		}
	
}

int GetNumberOfPoints(const PointCloud& pointCloud)
{
	return pointCloud.data.points.nCount;
}

T_Double GetXCoordinate(const PointCloud& pointCloud, int pointIndex)
{
	ASSERT_ON_TEST(pointIndex < pointCloud.data.points.nCount, "A missing point was requested from a features vector 2D");
	return pointCloud.data.points.arr[pointIndex].arr[0];
}

T_Double GetYCoordinate(const PointCloud& pointCloud, int pointIndex)
{
	ASSERT_ON_TEST(pointIndex < pointCloud.data.points.nCount, "A missing point was requested from a features vector 2D");
	return pointCloud.data.points.arr[pointIndex].arr[1];
}

T_Double GetZCoordinate(const PointCloud& pointCloud, int pointIndex)
{
	ASSERT_ON_TEST(pointIndex < pointCloud.data.points.nCount, "A missing point was requested from a features vector 2D");
	return pointCloud.data.points.arr[pointIndex].arr[2];
}

void RemovePoints(PointCloud& pointCloud, std::vector<BaseTypesWrapper::T_UInt32> pointIndexOrderedList)
	{
	BaseTypesWrapper::T_UInt32 elementsToRemove = pointIndexOrderedList.size();
	if ( elementsToRemove == 0)
		{
		return;
		}
	//Checking that input is ordered correctly.
	static const std::string errorMessage = "Remove Points in PointCloud error, the second input was not an ORDERED list or some index is not within range";
	for(int listIndex = 1; listIndex < elementsToRemove-1; listIndex++)
		{
		ASSERT( pointIndexOrderedList.at(listIndex-1) < pointIndexOrderedList.at(listIndex), errorMessage);
		ASSERT(	pointIndexOrderedList.at(listIndex) < pointIndexOrderedList.at(listIndex+1), errorMessage);
		}
	ASSERT( pointIndexOrderedList.at(elementsToRemove-1) < pointCloud.data.points.nCount, errorMessage);
	BaseTypesWrapper::T_UInt32 firstIndex = pointIndexOrderedList.at(0);
	ASSERT(firstIndex >= 0, errorMessage);

	//Removing elements 
	BaseTypesWrapper::T_UInt32 nextIndexToRemove = 1;
	BaseTypesWrapper::T_UInt32 currentGap = 1;
	for(int pointIndex = firstIndex; pointIndex < pointCloud.data.points.nCount - elementsToRemove; pointIndex++)
		{
		if (nextIndexToRemove < elementsToRemove && pointIndex+currentGap == pointIndexOrderedList.at(nextIndexToRemove))
			{
			currentGap++;
			nextIndexToRemove++;
			pointIndex--; //This is to not allow the map index to step forward in the next iteration;
			}
		else
			{
			pointCloud.data.points.arr[pointIndex].arr[0] = pointCloud.data.points.arr[pointIndex+currentGap].arr[0];
			pointCloud.data.points.arr[pointIndex].arr[1] = pointCloud.data.points.arr[pointIndex+currentGap].arr[1];
			pointCloud.data.points.arr[pointIndex].arr[2] = pointCloud.data.points.arr[pointIndex+currentGap].arr[2];
			}
		}
	pointCloud.data.points.nCount -= elementsToRemove;
	}

BitStream ConvertToBitStream(const PointCloud& pointCloud)
	CONVERT_TO_BIT_STREAM(pointCloud, asn1SccPointcloud_REQUIRED_BYTES_FOR_ENCODING, asn1SccPointcloud_Encode)

void ConvertFromBitStream(BitStream bitStream, PointCloud& pointCloud)
	CONVERT_FROM_BIT_STREAM(bitStream, asn1SccPointcloud_REQUIRED_BYTES_FOR_ENCODING, pointCloud, asn1SccPointcloud_Decode)

}

/** @} */
