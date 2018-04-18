/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudMap.cpp
 * @date 18/04/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the PointCloudMap class
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
#include "PointCloudMap.hpp"
#include "Errors/Assert.hpp"
#include <pcl/filters/voxel_grid.h>

namespace dfpc_ci {

using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;
using namespace BaseTypesWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
PointCloudMap::PointCloudMap() :
	pointCloud( new pcl::PointCloud<pcl::PointXYZ>() )
	{

	}

PointCloudMap::~PointCloudMap()
	{

	}

void PointCloudMap::AddPointCloud(PointCloudConstPtr pointCloudInput, VisualPointFeatureVector3DConstPtr pointCloudFeaturesVector, Pose3DConstPtr cloudPoseInMap)
	{
	if (featuresList.size() == 0 && GetNumberOfPoints(*pointCloudFeaturesVector) > 0)
		{
		descriptorLength = GetNumberOfDescriptorComponents(*pointCloudFeaturesVector, 0);
		}
	if (GetNumberOfPoints(*pointCloudFeaturesVector) > 0)
		{
		ASSERT( descriptorLength == GetNumberOfDescriptorComponents(*pointCloudFeaturesVector, 0), "PointCloudMap Error, Descriptor size mismatch");
		}
	ASSERT( GetVectorType(*pointCloudFeaturesVector) == ALL_POSITIONS_VECTOR, "PointCloudMap Error, added features vector does not contain all references");

	//Try new method only filter manually features vector and use voxel grid for point cloud.

	for(unsigned featureIndex = 0; featureIndex < GetNumberOfPoints(*pointCloudFeaturesVector); featureIndex++)
		{
		pcl::PointXYZ point(GetXCoordinate(*pointCloudFeaturesVector, featureIndex),GetYCoordinate(*pointCloudFeaturesVector, featureIndex),GetZCoordinate(*pointCloudFeaturesVector, featureIndex));
		pcl::PointXYZ transformedPoint = TransformPoint(point, cloudPoseInMap);
		bool pointAdded = AddPointToMap(transformedPoint);
		if (pointAdded)
			{
			FeaturePoint featurePoint;
			featurePoint.pointCloudIndex = pointCloud->points.size()-1;
			for(unsigned componentIndex = 0; componentIndex < descriptorLength; componentIndex++)
				{
				featurePoint.descriptor[componentIndex] = GetDescriptorComponent(*pointCloudFeaturesVector, featureIndex, componentIndex);
				}
			featuresList.push_back(featurePoint);
			}
		}

	for(uint64_t pointIndex = 0; pointIndex < GetNumberOfPoints(*pointCloudInput); pointIndex++)
		{
		pcl::PointXYZ point( GetXCoordinate(*pointCloudInput, pointIndex), GetYCoordinate(*pointCloudInput, pointIndex), GetZCoordinate(*pointCloudInput, pointIndex));
		pcl::PointXYZ transformedPoint = TransformPoint(point, cloudPoseInMap);
		AddPointToMap(transformedPoint);						
		}
	}

PointCloudConstPtr PointCloudMap::GetScenePointCloud(Pose3DConstPtr origin,  float radius)
	{
	pcl::PointXYZ pclOrigin( GetXPosition(*origin), GetYPosition(*origin), GetZPosition(*origin));
	PointCloudPtr pointCloudOutput = NewPointCloud();

	for(uint64_t pointIndex = 0; pointIndex < pointCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ cloudPoint = pointCloud->points.at(pointIndex);
		float pointToOriginDistance = PointDistance(pclOrigin, cloudPoint);
		if ( pointToOriginDistance <= radius )
			{
			AddPoint(*pointCloudOutput, cloudPoint.x, cloudPoint.y, cloudPoint.z);
			}
		}

	return pointCloudOutput;
	}

VisualPointFeatureVector3DConstPtr PointCloudMap::GetSceneFeaturesVector(Pose3DConstPtr origin,  float radius)
	{
	DEBUG_PRINT_TO_LOG("Number of points in stored cloud:", pointCloud->points.size());
	DEBUG_PRINT_TO_LOG("Number of features in stored cloud:", featuresList.size());

	pcl::PointXYZ pclOrigin( GetXPosition(*origin), GetYPosition(*origin), GetZPosition(*origin));
	VisualPointFeatureVector3DPtr featuresVector = NewVisualPointFeatureVector3D();

	uint64_t featureCounter = 0;
	for(uint64_t featureIndex = 0; featureIndex < featuresList.size(); featureIndex++)
		{
		FeaturePoint& featurePoint = featuresList.at(featureIndex); 
		pcl::PointXYZ cloudPoint = pointCloud->points.at( featurePoint.pointCloudIndex);
		float featureToOriginDistance = PointDistance(pclOrigin, cloudPoint);
		if ( featureToOriginDistance <= radius )
			{
			AddPoint(*featuresVector, cloudPoint.x, cloudPoint.y, cloudPoint.z);
			for(unsigned componentIndex = 0; componentIndex < descriptorLength; componentIndex++)
				{
				AddDescriptorComponent(*featuresVector, featureCounter, featurePoint.descriptor[componentIndex]);
				}
			featureCounter++;
			}
		}

	return featuresVector;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const float PointCloudMap::RESOLUTION = 1e-2;

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
float PointCloudMap::PointDistance(pcl::PointXYZ p, pcl::PointXYZ q)
	{
	float differenceX = p.x - q.x;
	float differenceY = p.y - q.y;
	float differenceZ = p.z - q.z;
	return std::sqrt( differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ);
	}

pcl::PointXYZ PointCloudMap::TransformPoint(pcl::PointXYZ point, PoseWrapper::Pose3DConstPtr cloudPoseInMap)
	{
	Eigen::Quaternion<float> rotation(GetWRotation(*cloudPoseInMap), GetXRotation(*cloudPoseInMap), GetYRotation(*cloudPoseInMap), GetZRotation(*cloudPoseInMap));
	Eigen::Translation<float, 3> translation( GetXPosition(*cloudPoseInMap), GetYPosition(*cloudPoseInMap), GetZPosition(*cloudPoseInMap));
	Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> affineTransform = rotation * translation;

	Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
	Eigen::Vector3f eigenTransformedPoint = affineTransform.inverse() * eigenPoint;
	pcl::PointXYZ transformedPoint;
	transformedPoint.x = eigenTransformedPoint.x();
	transformedPoint.y = eigenTransformedPoint.y();
	transformedPoint.z = eigenTransformedPoint.z();
	return transformedPoint;
	}

bool PointCloudMap::AddPointToMap(pcl::PointXYZ point)
	{
	bool noClosePoint = true;
	for(unsigned pointIndex = 0; pointIndex < pointCloud->points.size() && noClosePoint; pointIndex++)
		{
		pcl::PointXYZ cloudPoint = pointCloud->points.at(pointIndex);
		float distance = PointDistance(point, cloudPoint);
		noClosePoint = (distance > RESOLUTION);
		}

	if (noClosePoint)
		{
		pointCloud->points.push_back(point);
		}	

	return noClosePoint;
	}

bool PointCloudMap::IndexNotCotainedInVector(uint64_t index, VisualPointFeatureVector3DConstPtr pointCloudFeaturesVector)
	{
	for(unsigned featureIndex = 0; featureIndex < GetNumberOfPoints(*pointCloudFeaturesVector); featureIndex++)
		{
		uint64_t pointIndex = GetReferenceIndex(*pointCloudFeaturesVector, featureIndex);
		if (pointIndex == index)
			{
			return false;
			}
		}
	return true;
	}

}

/** @} */

