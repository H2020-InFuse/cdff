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
PointCloudTransformer::PointCloudTransformer() :
	pointCloud( new pcl::PointCloud<pcl::PointXYZ> ), transformedCloud( new pcl::PointCloud<pcl::PointXYZ> )
	{
	transformedCloudWasInitialized = false;
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
	InitTransformedCloud();
	ASSERT(minIndex >= 0 && minIndex <= maxIndex && maxIndex < transformedCloud->points.size(), "Error, indices out of range");

	pcl::PointCloud<pcl::PointXYZ>::Ptr newTransformedCloud( new pcl::PointCloud<pcl::PointXYZ> );	

	newTransformedCloud->points.resize( maxIndex - minIndex + 1);
	for(unsigned pointIndex = minIndex; pointIndex <= maxIndex; pointIndex++)
		{
		newTransformedCloud->points.at(pointIndex - minIndex) = transformedCloud->points.at(pointIndex);
		}
	
	transformedCloud = newTransformedCloud;
	}

void PointCloudTransformer::TransformCloud(float positionX, float positionY, float positionZ, float rotationX, float rotationY, float rotationZ, float rotationW)
	{
	InitTransformedCloud();
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
	InitTransformedCloud();
	std::normal_distribution<double> gaussianNoiseSource(mean, standardDeviation);
	
	for(unsigned pointIndex = 0; pointIndex < transformedCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ& point = transformedCloud->points.at(pointIndex);
		point.x += gaussianNoiseSource(randomEngine);
		point.y += gaussianNoiseSource(randomEngine);
		point.z += gaussianNoiseSource(randomEngine);
		}
	}

void PointCloudTransformer::SavePointCloud(std::string outputFilePath)
	{
	ASSERT(transformedCloudWasInitialized, "You did not apply any transform, there is no need to save this cloud");
	pcl::PLYWriter writer;
	writer.write(outputFilePath, *transformedCloud);
	}

void PointCloudTransformer::ViewPointCloud()
	{
	std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > cloudsList = { pointCloud, transformedCloud };
	Visualizers::PclVisualizer::Enable();
	Visualizers::PclVisualizer::ShowPointClouds(cloudsList);
	Visualizers::PclVisualizer::Disable();
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void PointCloudTransformer::InitTransformedCloud()
	{
	if (transformedCloudWasInitialized)
		{
		return;
		}

	transformedCloud->points.resize( pointCloud->points.size() );
	for(unsigned pointIndex = 0; pointIndex < pointCloud->points.size(); pointIndex++)
		{
		transformedCloud->points.at(pointIndex) = pointCloud->points.at(pointIndex);
		}
	transformedCloudWasInitialized = true;
	}

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
