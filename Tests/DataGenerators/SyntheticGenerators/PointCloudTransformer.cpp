/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudTransformer.cpp
 * @date 23/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the PointCloudTransformer class.
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
#include "PointCloudTransformer.hpp"
#include <stdlib.h>
#include <random>
#include <Errors/Assert.hpp>
#include <Visualizers/PclVisualizer.hpp>

namespace DataGenerators
{

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
PointCloudTransformer::PointCloudTransformer(bool enableVisualizer) :
	pointCloud( new pcl::PointCloud<pcl::PointXYZ> ), transformedCloud( new pcl::PointCloud<pcl::PointXYZ> )
	{
	if (enableVisualizer)
		{
		Visualizers::PclVisualizer::Enable();
		}
	}

PointCloudTransformer::~PointCloudTransformer()
	{

	}

void PointCloudTransformer::LoadPointCloud(std::string pointCloudFilePath)
	{
	pcl::io::loadPLYFile(pointCloudFilePath, *pointCloud);
	PRINT_TO_LOG("Number of points: ", (pointCloud->points.size()) );
	}

void PointCloudTransformer::Resize(unsigned minIndex, unsigned maxIndex)
	{
	ASSERT(minIndex >= 0 && minIndex <= maxIndex && maxIndex < pointCloud->points.size(), "Error, indices out of range");

	transformedCloud->points.resize( maxIndex - minIndex + 1);

	for(unsigned pointIndex = minIndex; pointIndex <= maxIndex; pointIndex++)
		{
		transformedCloud->points.at(pointIndex - minIndex) = pointCloud->points.at(pointIndex);
		}
	DEBUG_SHOW_POINT_CLOUD(transformedCloud);
	}

void PointCloudTransformer::TransformCloud(float positionX, float positionY, float positionZ, float rotationX, float rotationY, float rotationZ, float rotationW)
	{
	Eigen::Quaternion<float> rotation(rotationW, rotationX, rotationY, rotationZ);
	Eigen::Translation<float, 3> translation(positionX, positionY, positionZ);
	AffineTransform affineTransform = rotation * translation;
	
	for(unsigned pointIndex = 0; pointIndex < transformedCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ transformedPoint = TransformPoint( pointCloud->points.at(pointIndex), affineTransform );
		transformedCloud->points.at(pointIndex) = transformedPoint;
		}
	}

void PointCloudTransformer::AddGaussianNoise(float mean, float standardDeviation)
	{
	std::normal_distribution<double> gaussianNoiseSource(mean, standardDeviation);
	
	for(unsigned pointIndex = 0; pointIndex < transformedCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ& point = transformedCloud->points.at(pointIndex);
		point.x += gaussianNoiseSource(randomEngine);
		point.y += gaussianNoiseSource(randomEngine);
		point.z += gaussianNoiseSource(randomEngine);
		}
	DEBUG_SHOW_POINT_CLOUD(transformedCloud);
	}

void PointCloudTransformer::SavePointCloud(std::string outputFilePath)
	{
	pcl::PLYWriter writer;
	writer.write(outputFilePath, *transformedCloud);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */

pcl::PointXYZ PointCloudTransformer::TransformPoint(const pcl::PointXYZ& point, const AffineTransform& affineTransform)
	{
	Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
	Eigen::Vector3f eigenTransformedPoint = affineTransform * eigenPoint;
	pcl::PointXYZ transformedPoint;
	transformedPoint.x = eigenTransformedPoint.x();
	transformedPoint.y = eigenTransformedPoint.y();
	transformedPoint.z = eigenTransformedPoint.z();
	return transformedPoint;
	}

}
/** @} */