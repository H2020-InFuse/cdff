/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file RegularityTester.cpp
 * @date 02/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the RegularityTester class.
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
#include "RegularityTester.hpp"
#include<pcl/io/ply_io.h>
#include <ctime>

using namespace CDFF::DFN::FeaturesExtraction3D;
using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;
using namespace PoseWrapper;

#define DELETE_IF_NOT_NULL(pointer) \
	if (pointer != NULL) \
		{ \
		delete(pointer); \
		}

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
RegularityTester::RegularityTester(std::string configurationFilePath, std::string pointCloudFilePath, AverageSeparationType averageSeparationType, FeaturesExtraction3DInterface* dfn) 
	{
	this->configurationFilePath = configurationFilePath;
	this->pointCloudFilePath = pointCloudFilePath;
	this->averageSeparationType = averageSeparationType;
	this->dfn = dfn;

	inputCloud = NULL;
	outputFeaturesVector = NULL;

	ConfigureDfn();
	LoadPointCloud();
	}

RegularityTester::~RegularityTester()
	{
	DELETE_IF_NOT_NULL(inputCloud);
	DELETE_IF_NOT_NULL(outputFeaturesVector);
	}

void RegularityTester::ExecuteDfn()
	{
	dfn->pointcloudInput(*inputCloud);

	clock_t beginTime = clock();
	dfn->process();
	clock_t endTime = clock();
	float processingTime = float(endTime - beginTime) / CLOCKS_PER_SEC;
	PRINT_TO_LOG("Processing took (seconds): ", processingTime);

	DELETE_IF_NOT_NULL(outputFeaturesVector);
	VisualPointFeatureVector3DPtr newOutputFeaturesVector = NewVisualPointFeatureVector3D();
	Copy( dfn->featuresOutput(), *newOutputFeaturesVector);
	outputFeaturesVector = newOutputFeaturesVector;

	PRINT_TO_LOG("Number of keypoints extracted is", GetNumberOfPoints(*outputFeaturesVector));
	}

bool RegularityTester::IsOutputRegular(float regularity)
	{
	if ( GetNumberOfPoints(*outputFeaturesVector) <= 1)
		{
		return true;
		}

	float averageSeparation = ComputeAverageSeparation();
	float distanceThreshold = averageSeparation * regularity;
	PRINT_TO_LOG("Average Separation is", averageSeparation);
	PRINT_TO_LOG("Distance Threshold is", distanceThreshold);

	ComputeClustersList(distanceThreshold);
	PRINT_TO_LOG("Number of computed clusters is", clustersList.size());
	PRINT_TO_LOG("Average number of points in cluster is", averageNumberOfPointsInCluster);

	bool regular = ThereAreNoGaps(distanceThreshold);
	regular = ThereAreNoClusters(distanceThreshold) && regular;

	return regular;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void RegularityTester::LoadPointCloud()
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr basePclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile(pointCloudFilePath, *basePclCloud);

	DELETE_IF_NOT_NULL(inputCloud);	
	inputCloud = pointCloudConverter.Convert(basePclCloud);
	}

void RegularityTester::ConfigureDfn()
	{
	dfn->setConfigurationFile(configurationFilePath);
	dfn->configure();
	}

float RegularityTester::ComputeAverageSeparation()
	{
	switch(averageSeparationType)
		{
		case POINTS_PAIR_DISTANCE: 
			{
			PRINT_TO_LOG("Averaging distance between each pairs of points", "");
			return ComputeAveragePointsPairDistance();
			}
		case CLOSEST_NEIGHBOUR_DISTANCE:
			{
			PRINT_TO_LOG("Averaging distances between a point and its closest neighbour", "");
			return ComputeAverageClosestNeighbourDistance();
			}
		default:
			{
			ASSERT(false, "Unhandled average separation type");
			}
		}
	return 0;
	}

float RegularityTester::ComputeAveragePointsPairDistance()
	{
	int numberOfPoints = GetNumberOfPoints(*outputFeaturesVector);
	double sumOfDistances = 0;

	//This loop sums the distance between each pairs of points
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		for(int secondPointIndex = pointIndex+1; secondPointIndex < numberOfPoints; secondPointIndex++)
			{	
			sumOfDistances += ComputeDistance(pointIndex, secondPointIndex);
			}	
		}

	double pointPairsNumber = numberOfPoints * (numberOfPoints-1) / 2;	
	return (sumOfDistances / pointPairsNumber);
	}

float RegularityTester::ComputeAverageClosestNeighbourDistance()
	{
	int numberOfPoints = GetNumberOfPoints(*outputFeaturesVector);
	double sumOfDistances = 0;

	//The loops sums the closest distance between a point and each neighbour
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		float closestNeighbourDistance;
		
		//Initializing closest distance
		if (pointIndex == 0)
			{
			closestNeighbourDistance = ComputeDistance(pointIndex, pointIndex+1);
			}
		else
			{
			closestNeighbourDistance = ComputeDistance(pointIndex-1, pointIndex);
			}

		//Search for the closest distance of the point with its neighbours
		for(int secondPointIndex = 0; secondPointIndex < numberOfPoints; secondPointIndex++)	
			{
			float distance = ComputeDistance(secondPointIndex, pointIndex);
			if (pointIndex != secondPointIndex && distance < closestNeighbourDistance)
				{
				closestNeighbourDistance = distance;
				}
			}
		sumOfDistances += closestNeighbourDistance;
		}

	return (sumOfDistances / numberOfPoints);
	}

void RegularityTester::ComputeClustersList(float distanceThreshold)
	{
	clustersList.clear();
	int numberOfPoints = GetNumberOfPoints(*outputFeaturesVector);

	//pointsAreInCluster will contain a bool for each feature point. This boolean value represents whether the point has already been put in some cluster.
	std::vector<bool> pointsAreInCluster;
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		pointsAreInCluster.push_back(false);
		}

	//The following loop iterates as long as there is some point which does not belong to some cluster
	std::vector<bool>::iterator nextPointIterator = std::find(pointsAreInCluster.begin(), pointsAreInCluster.end(), false);
	while(nextPointIterator != pointsAreInCluster.end())
		{
		Cluster newCluster;
		int nextPointIndex = nextPointIterator - pointsAreInCluster.begin();

		newCluster.push_back(nextPointIndex);
		pointsAreInCluster.at(nextPointIndex) = true;
		CompleteCluster(newCluster, pointsAreInCluster, distanceThreshold);

		clustersList.push_back(newCluster);
		nextPointIterator = std::find(pointsAreInCluster.begin(), pointsAreInCluster.end(), false);
		}

	averageNumberOfPointsInCluster = ComputeAverageNumberOfPointsInCluster();
	}

void RegularityTester::CompleteCluster(Cluster& cluster, std::vector<bool>& pointsAreInOneCluster, float distanceThreshold)
	{
	int numberOfPoints = GetNumberOfPoints(*outputFeaturesVector);

	//This loop will expand the cluster as long as its distance size does not exceed the distanceThreshold. Only points which do not belong to other clusters will be used in the expansion.
	float clusterSize = ComputeClusterSize(cluster);	
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		if ( pointsAreInOneCluster.at(pointIndex) )
			{
			continue;
			}

		float wouldBeSize = ComputeWouldBeSizeIfPointWasAdded(cluster, pointIndex, clusterSize);
		if (wouldBeSize > distanceThreshold)
			{
			continue;
			}
			
		cluster.push_back(pointIndex);
		clusterSize = wouldBeSize;
		pointsAreInOneCluster.at(pointIndex) = true;
		}			
	}

bool RegularityTester::ThereAreNoGaps(float distanceThreshold)
	{
	if (clustersList.size() <= 1)
		{
		return true;
		}

	float largestGap = 0;
	//This loops iterates through all the cluster, for each cluster it computes the closest cluster neighbour.
	//If the distance to the closest cluster neighbour is greater than the input distanceThreshold, the gap is found and function gives true as output.
	for(int clusterIndex1 = 0; clusterIndex1 < clustersList.size(); clusterIndex1++)
		{
		int clusterSize = clustersList.at(clusterIndex1).size();

		//This line filter isolated points (i.e. cluster with less than five points). I'm not sure whether we should filter the isolated points though.
		if (clusterSize < 5)
			{
			continue;
			}

		//Initialization of the closest cluster
		float closestClusterDistance;
		if (clusterIndex1 == 0)
			{
			closestClusterDistance = ComputeDistance( clustersList.at(clusterIndex1), clustersList.at(clusterIndex1+1) );
			}
		else
			{
			closestClusterDistance = ComputeDistance( clustersList.at(clusterIndex1-1), clustersList.at(clusterIndex1) );
			}

		//Search for the closest cluster
		for(int clusterIndex2 = 0; clusterIndex2 < clustersList.size(); clusterIndex2++)
			{
			float clusterDistance = ComputeDistance( clustersList.at(clusterIndex1), clustersList.at(clusterIndex2) );
			if (clusterIndex1 != clusterIndex2 && clusterDistance < closestClusterDistance)
				{
				closestClusterDistance = clusterDistance;
				}
			}

		if (largestGap < closestClusterDistance)
			{
			largestGap = closestClusterDistance;
			}
		//Chcking distance threshold
		if (closestClusterDistance > distanceThreshold)
			{
			PRINT_TO_LOG("We found an isolated cluster containing this number of points: ", clusterSize);
			}
		}

	PRINT_TO_LOG("The largest gap is:", largestGap);
	return (largestGap <= distanceThreshold);
	}

bool RegularityTester::ThereAreNoClusters(float distanceThreshold)
	{
	float largestClusterSize = 0;
	for(int clusterIndex = 0; clusterIndex < clustersList.size(); clusterIndex++)
		{
		float clusterSize = ComputeClusterSize( clustersList.at(clusterIndex) );
		if (largestClusterSize < clusterSize)
			{
			largestClusterSize = clusterSize;
			}
		}
	PRINT_TO_LOG("Largest Cluster has size:", largestClusterSize);
	return (largestClusterSize <= distanceThreshold);
	}

float RegularityTester::ComputeDistance(int pointIndex1, int pointIndex2)
	{
	const float& x1 = GetXCoordinate(*outputFeaturesVector, pointIndex1);
	const float& y1 = GetYCoordinate(*outputFeaturesVector, pointIndex1);	
	const float& z1 = GetZCoordinate(*outputFeaturesVector, pointIndex1);	
	const float& x2 = GetXCoordinate(*outputFeaturesVector, pointIndex2);
	const float& y2 = GetYCoordinate(*outputFeaturesVector, pointIndex2);
	const float& z2 = GetZCoordinate(*outputFeaturesVector, pointIndex2);
	return std::sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2) );
	}

float RegularityTester::ComputeDistance(const Cluster& cluster1, const Cluster& cluster2)
	{
	ASSERT(cluster1.size() > 0 && cluster2.size() > 0, "Error: trying to compute cluster distance between empty clusters");
	float distance = ComputeDistance(cluster1.at(0), cluster2.at(0));
	
	//Computing the smallest pairwise distance between two points: one point for each cluster.
	for(int clusterIndex1 = 0; clusterIndex1 < cluster1.size(); clusterIndex1++)
		{
		for (int clusterIndex2 = 0; clusterIndex2 < cluster2.size(); clusterIndex2++)
			{
			float pointsDistance = ComputeDistance(cluster1.at(clusterIndex1), cluster2.at(clusterIndex2));
			if (distance > pointsDistance)
				{
				distance = pointsDistance;
				}
			}
		}

	return distance;
	}

float RegularityTester::ComputeClusterSize(const Cluster& cluster)
	{
	float distance = 0;
	//Computing the largest pairwise distance between two points of the cluster
	for(int clusterIndex1 = 0; clusterIndex1 < cluster.size(); clusterIndex1++)
		{
		for (int clusterIndex2 = 0; clusterIndex2 < cluster.size(); clusterIndex2++)
			{
			float pointsDistance = ComputeDistance(cluster.at(clusterIndex1), cluster.at(clusterIndex2));
			if (distance < pointsDistance)
				{
				distance = pointsDistance;
				}
			}
		}

	return distance;
	}

float RegularityTester::ComputeWouldBeSizeIfPointWasAdded(const Cluster& cluster, int pointIndex, float currentSize)
	{
	float distance = currentSize;
	
	//The loop computes the maximum distance between the added point and a point in the cluster. If this distance is higher than the currentDistance than it becomes the new cluster size.
	for(int clusterIndex = 0; clusterIndex < cluster.size(); clusterIndex++)
		{
		float pointsDistance = ComputeDistance(cluster.at(clusterIndex), pointIndex);
		if (distance < pointsDistance)
			{
			distance = pointsDistance;
			}
		}
	return distance;	
	}

float RegularityTester::ComputeAverageNumberOfPointsInCluster()
	{
	float numberOfPoints = GetNumberOfPoints(*outputFeaturesVector);
	float numberOfClusters = clustersList.size();
	return numberOfPoints / numberOfClusters;
	}

/** @} */
