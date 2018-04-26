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
	resolution = DEFAULT_RESOLUTION;
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
	ASSERT( GetVectorType(*pointCloudFeaturesVector) == ALL_POSITIONS_VECTOR, "PointCloudMap Error, added features vector does not contain all positions");

	AffineTransform affineTransform = ConvertCloudPoseToInversionTransform(cloudPoseInMap);
	AddFeatureCloud(pointCloudFeaturesVector, affineTransform);
	AddPointCloud(pointCloudInput, affineTransform);
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
		FeaturePoint featurePoint = featuresList.at(featureIndex); 
		float featureToOriginDistance = PointDistance(pclOrigin, featurePoint.point);
		if ( featureToOriginDistance <= radius )
			{
			AddPoint(*featuresVector, featurePoint.point.x, featurePoint.point.y, featurePoint.point.z);
			for(unsigned componentIndex = 0; componentIndex < descriptorLength; componentIndex++)
				{
				AddDescriptorComponent(*featuresVector, featureCounter, featurePoint.descriptor[componentIndex]);
				}
			featureCounter++;
			}
		}

	return featuresVector;
	}

void PointCloudMap::SetResolution(float resolution)
	{
	this->resolution = resolution;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const float PointCloudMap::DEFAULT_RESOLUTION = 0.01;

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void PointCloudMap::AddPointCloud(PointCloudConstPtr pointCloudInput, const AffineTransform& affineTransform)
	{
	unsigned previousPointCloudSize = pointCloud->points.size();
	unsigned numberOfNewPoints = GetNumberOfPoints(*pointCloudInput);
	pointCloud->points.resize( previousPointCloudSize + numberOfNewPoints );

	for(unsigned pointIndex = 0; pointIndex < numberOfNewPoints; pointIndex++)
		{
		pcl::PointXYZ newPoint( GetXCoordinate(*pointCloudInput, pointIndex), GetYCoordinate(*pointCloudInput, pointIndex), GetZCoordinate(*pointCloudInput, pointIndex) );
		pcl::PointXYZ transformedNewPoint = TransformPoint(newPoint, affineTransform);
		pointCloud->points.at(pointIndex + previousPointCloudSize) = transformedNewPoint;		
		}

	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(pointCloud);
	grid.setLeafSize(resolution, resolution, resolution);
	grid.filter(*pointCloud);
	}

void PointCloudMap::AddFeatureCloud(VisualPointFeatureVector3DConstPtr pointCloudFeaturesVector, const AffineTransform& affineTransform)
	{
	for(unsigned featureIndex = 0; featureIndex < GetNumberOfPoints(*pointCloudFeaturesVector); featureIndex++)
		{
		pcl::PointXYZ point(GetXCoordinate(*pointCloudFeaturesVector, featureIndex),GetYCoordinate(*pointCloudFeaturesVector, featureIndex),GetZCoordinate(*pointCloudFeaturesVector, featureIndex));
		pcl::PointXYZ transformedPoint = TransformPoint(point, affineTransform);
		if (NoCloseFeature(transformedPoint))
			{
			FeaturePoint featurePoint;
			featurePoint.point = transformedPoint;
			for(unsigned componentIndex = 0; componentIndex < descriptorLength; componentIndex++)
				{
				featurePoint.descriptor[componentIndex] = GetDescriptorComponent(*pointCloudFeaturesVector, featureIndex, componentIndex);
				}
			featuresList.push_back(featurePoint);			
			}
		}
	}

PointCloudMap::AffineTransform PointCloudMap::ConvertCloudPoseToInversionTransform(PoseWrapper::Pose3DConstPtr cloudPoseInMap)
	{
	Eigen::Quaternion<float> rotation(GetWRotation(*cloudPoseInMap), GetXRotation(*cloudPoseInMap), GetYRotation(*cloudPoseInMap), GetZRotation(*cloudPoseInMap));
	Eigen::Translation<float, 3> translation( GetXPosition(*cloudPoseInMap), GetYPosition(*cloudPoseInMap), GetZPosition(*cloudPoseInMap));
	AffineTransform affineTransform = rotation * translation;
	return affineTransform.inverse();
	}

pcl::PointXYZ PointCloudMap::TransformPoint(const pcl::PointXYZ& point, const AffineTransform& affineTransform)
	{
	Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
	Eigen::Vector3f eigenTransformedPoint = affineTransform * eigenPoint;
	pcl::PointXYZ transformedPoint;
	transformedPoint.x = eigenTransformedPoint.x();
	transformedPoint.y = eigenTransformedPoint.y();
	transformedPoint.z = eigenTransformedPoint.z();
	return transformedPoint;
	}

float PointCloudMap::PointDistance(const pcl::PointXYZ& p, const pcl::PointXYZ& q)
	{
	float differenceX = p.x - q.x;
	float differenceY = p.y - q.y;
	float differenceZ = p.z - q.z;
	return std::sqrt( differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ);
	}

bool PointCloudMap::NoCloseFeature(const pcl::PointXYZ& point)
	{
	for(unsigned featureIndex = 0; featureIndex < featuresList.size(); featureIndex++)
		{
		float distance = PointDistance(point, featuresList.at(featureIndex).point);
		if (distance < resolution)
			{
			return false;
			}
		}
	return true;
	}

}

/** @} */
