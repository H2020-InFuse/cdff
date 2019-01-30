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
#include <Visualizers/PCLVisualizer.hpp>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

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

	AffineTransform affineTransform = translation * rotation;
	
	for(unsigned pointIndex = 0; pointIndex < transformedCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ transformedPoint = TransformPoint( pointCloud->points.at(pointIndex), affineTransform );
		transformedCloud->points.at(pointIndex) = transformedPoint;
		}
	}

void PointCloudTransformer::TransformCamera(float positionX, float positionY, float positionZ, float rotationX, float rotationY, float rotationZ, float rotationW)
	{
	InitTransformedCloud();
	Eigen::Quaternion<float> rotation(rotationW, rotationX, rotationY, rotationZ);
	Eigen::Translation<float, 3> translation(-positionX, -positionY, -positionZ);

	AffineTransform affineTransform = InvertQuaternion(rotation) * translation;
	
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

void PointCloudTransformer::RemoveOutliers()
	{
	InitTransformedCloud();	
	if ( transformedCloud->points.size() == 0)
		{
		return;
		}

	float totalDimension = 0;
	for(unsigned pointIndex = 0; pointIndex < transformedCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ& point = transformedCloud->points.at(pointIndex);
		if ( point.x == point.x && point.y == point.y && point.z == point.z )
			{
			totalDimension += (std::abs(point.x) + std::abs(point.y) + std::abs(point.z));
			}
		}
	float averageDimension = totalDimension / (3 * transformedCloud->points.size());

	pcl::PointCloud<pcl::PointXYZ>::Ptr newTransformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(unsigned pointIndex = 0; pointIndex < transformedCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = transformedCloud->points.at(pointIndex);
		if ( point.x == point.x && point.y == point.y && point.z == point.z && 
			std::abs(point.x) <= 20 * averageDimension && std::abs(point.y) <= 20 * averageDimension && std::abs(point.z) <= 20 * averageDimension)
			{
			newTransformedCloud->points.push_back(point);
			}
		}

	std::stringstream message;
	message << "Remaining points: " << newTransformedCloud->points.size() << "/" << transformedCloud->points.size();
	PRINT_TO_LOG(message.str(), "");
	transformedCloud = newTransformedCloud;
	}

void PointCloudTransformer::SavePointCloud(std::string outputFilePath)
	{
	ASSERT(transformedCloudWasInitialized, "You did not apply any transform, there is no need to save this cloud");
	pcl::PLYWriter writer;
	writer.write(outputFilePath, *transformedCloud, true);
	}

void PointCloudTransformer::ViewPointCloud()
	{
	std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > cloudsList = { pointCloud, transformedCloud };
	Visualizers::PclVisualizer::Enable();
	Visualizers::PclVisualizer::ShowPointClouds(cloudsList);
	Visualizers::PclVisualizer::Disable();
	}

void PointCloudTransformer::Rescale(float scale)
	{
	InitTransformedCloud();
	if ( transformedCloud->points.size() == 0)
		{
		return;
		}

	for(unsigned pointIndex = 0; pointIndex < transformedCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ& point = transformedCloud->points.at(pointIndex);
		point.x = point.x * scale;
		point.y = point.y * scale;
		point.z = point.z * scale;
		}	
	}

void PointCloudTransformer::ShowWithAddedCloud(std::string cloudFilePath, float positionX, float positionY, float positionZ, float rotationX, float rotationY, float rotationZ, float rotationW)
	{
	InitTransformedCloud();
	pcl::PointCloud<pcl::PointXYZ>::Ptr addedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile(cloudFilePath, *addedCloud);

	Eigen::Quaternion<float> rotation(rotationW, rotationX, rotationY, rotationZ);
	Eigen::Translation<float, 3> translation(positionX, positionY, positionZ);
	AffineTransform affineTransform = translation * rotation;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr addedTransformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	addedTransformedCloud->points.resize( addedCloud->points.size() );
	for(unsigned pointIndex = 0; pointIndex < addedCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ transformedPoint = TransformPoint( addedCloud->points.at(pointIndex), affineTransform );
		addedTransformedCloud->points.at(pointIndex) = transformedPoint;
		}

	std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > cloudsList = { transformedCloud, addedTransformedCloud };
	Visualizers::PclVisualizer::Enable();
	Visualizers::PclVisualizer::ShowPointClouds(cloudsList);
	Visualizers::PclVisualizer::Disable();
	}

void PointCloudTransformer::Reduce(int targetNumberOfPoints)
	{
	InitTransformedCloud();

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (transformedCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr newTransformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::srand(std::time(nullptr));

	int numberOfLeftoverPoints = transformedCloud->points.size();
	int numberOfPointsToKeep = newTransformedCloud->points.size();
	while (numberOfLeftoverPoints + numberOfPointsToKeep > targetNumberOfPoints && numberOfPointsToKeep < targetNumberOfPoints)
		{
		int pointsToRemoveForEachKeptPoint = numberOfLeftoverPoints/(targetNumberOfPoints - numberOfPointsToKeep);	
		std::vector<int> indexList(pointsToRemoveForEachKeptPoint+1);
		std::vector<float> squaredDistancesList(pointsToRemoveForEachKeptPoint+1);

		int randomNumber = std::round(static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * static_cast<float>(transformedCloud->points.size()-1));

		pcl::PointXYZ searchPoint = transformedCloud->points.at( randomNumber );
		kdtree.nearestKSearch (searchPoint, pointsToRemoveForEachKeptPoint+1, indexList, squaredDistancesList);

		newTransformedCloud->points.push_back(searchPoint);
		std::sort(indexList.begin(), indexList.end());

		for(int i=pointsToRemoveForEachKeptPoint; i>=0; i--)
			{
			int pointToRemove = indexList.at(i);
			std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::iterator elementToRemove = transformedCloud->points.begin() + pointToRemove;
			transformedCloud->points.erase(elementToRemove);
			}

		numberOfLeftoverPoints = transformedCloud->points.size();
		numberOfPointsToKeep = newTransformedCloud->points.size();		
		}

	for( int pointIndex=0; pointIndex < numberOfLeftoverPoints; pointIndex++)
		{
		pcl::PointXYZ newPoint = transformedCloud->points.at(pointIndex);		
		newTransformedCloud->points.push_back(newPoint);
		}
	transformedCloud = newTransformedCloud;
	}

void PointCloudTransformer::RemoveDominantPlane(float inlierDistance)
	{
	InitTransformedCloud();
	
	pcl::SACSegmentation<pcl::PointXYZ> ransacSegmentation;
	ransacSegmentation.setOptimizeCoefficients (true);
	ransacSegmentation.setModelType (pcl::SACMODEL_PLANE);
	ransacSegmentation.setMethodType (pcl::SAC_RANSAC);
	ransacSegmentation.setDistanceThreshold (inlierDistance);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	ransacSegmentation.setInputCloud (transformedCloud);
	ransacSegmentation.segment (*inliers, *coefficients);

	pcl::PointCloud<pcl::PointXYZ>::Ptr leftoverCloud( new pcl::PointCloud<pcl::PointXYZ>() );
	int leftoverIndex = 0;	
	for(int indexIndex = 0; indexIndex < inliers->indices.size(); indexIndex++)
		{
		int pointIndex = inliers->indices.at(indexIndex);
		while (leftoverIndex < pointIndex)
			{
			pcl::PointXYZ leftoverPoint= transformedCloud->points.at(leftoverIndex);
			leftoverCloud->points.push_back(leftoverPoint);
			leftoverIndex++;
			}
		leftoverIndex++;
		}
	transformedCloud = leftoverCloud;
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

Eigen::Quaternion<float> PointCloudTransformer::InvertQuaternion(Eigen::Quaternion<float> input)
	{
	float sum = std::abs( input.x() * input.x() + input.y() * input.y() + input.z() * input.z() + input.w() * input.w() );
	return Eigen::Quaternion<float>( input.w() / sum, -input.x() / sum, -input.y() / sum, -input.z() / sum); 
	}

}
/** @} */
