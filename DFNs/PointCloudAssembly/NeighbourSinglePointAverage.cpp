/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "NeighbourSinglePointAverage.hpp"

#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <stdlib.h>
#include <fstream>

using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;
using namespace PoseWrapper;

namespace CDFF
{
namespace DFN
{
namespace PointCloudAssembly
{

NeighbourSinglePointAverage::NeighbourSinglePointAverage() :
	storedCloud(NULL),
	assembledCloud( boost::make_shared< pcl::PointCloud<pcl::PointXYZ> >(*( new pcl::PointCloud<pcl::PointXYZ> )) )
{
        parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<float>("GeneralParameters", "MaxNeighbourDistance", parameters.maxNeighbourDistance, DEFAULT_PARAMETERS.maxNeighbourDistance);
	parametersHelper.AddParameter<bool>("GeneralParameters", "IgnoreUnmatchedPointsOnFirstCloud", parameters.ignoreUnmatchedPointsOnFirstCloud, DEFAULT_PARAMETERS.ignoreUnmatchedPointsOnFirstCloud);
	parametersHelper.AddParameter<bool>("GeneralParameters", "IgnoreUnmatchedPointsOnSecondCloud", parameters.ignoreUnmatchedPointsOnSecondCloud, DEFAULT_PARAMETERS.ignoreUnmatchedPointsOnSecondCloud);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseIncrementalMode", parameters.useIncrementalMode, DEFAULT_PARAMETERS.useIncrementalMode);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseDistanceFilter", parameters.useDistanceFilter, DEFAULT_PARAMETERS.useDistanceFilter);

	configurationFilePath = "";

	SetPosition(inViewCenter, 0, 0, 0);
	inViewRadius = 100;
}

NeighbourSinglePointAverage::~NeighbourSinglePointAverage()
{
}

void NeighbourSinglePointAverage::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();

	if (parameters.useIncrementalMode && storedCloud == NULL)
		{
		storedCloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZ> >(*( new pcl::PointCloud<pcl::PointXYZ> ));
		}
}

void NeighbourSinglePointAverage::process()
{
	if (!parameters.useIncrementalMode)
		{
		firstCloud = pointCloudToPclPointCloud.Convert(&inFirstPointCloud);
		secondCloud = pointCloudToPclPointCloud.Convert(&inSecondPointCloud);
		}
	else
		{
		firstCloud = storedCloud;
		secondCloud = pointCloudToPclPointCloud.Convert(&inFirstPointCloud);
		}

	ComputeCorrespondenceMap();
	ComputeReplacementPoints();
	AssemblePointCloud(); 
	PrepareOutAssembledPointCloud();

	if (parameters.useIncrementalMode)
		{
		storedCloud->points.resize(0);
		*storedCloud += *assembledCloud;
		}
}

const NeighbourSinglePointAverage::NeighbourSinglePointAverageOptionsSet NeighbourSinglePointAverage::DEFAULT_PARAMETERS
{
	/*.maxNeighbourDistance =*/ 0.01,
	/*.ignoreUnmatchedPointsOnFirstCloud =*/ false,
	/*.ignoreUnmatchedPointsOnSecondCloud =*/ false,
	/*.useIncrementalMode =*/ false,
	/*.useDistanceFilter =*/ false
};

void NeighbourSinglePointAverage::ComputeCorrespondenceMap()
	{
	correspondenceMap.clear();

	if (firstCloud->points.size() == 0)
		{
		return;
		}

	pcl::KdTreeFLANN<pcl::PointXYZ> searchTree;
	searchTree.setInputCloud(firstCloud);

	std::vector<int> indexList;
	std::vector<float> squaredDistanceList;

	int numberOfSecondPoints = secondCloud->points.size();
	for(int pointIndex = 0; pointIndex < numberOfSecondPoints; pointIndex++)
		{
		pcl::PointXYZ searchPoint = secondCloud->points.at(pointIndex);
		if (searchTree.radiusSearch(searchPoint, parameters.maxNeighbourDistance, indexList, squaredDistanceList) > 0)
			{
			Correspondence correspondence;
			correspondence.firstIndex = indexList.at(0);
			correspondence.secondIndex = pointIndex;
			correspondence.squaredDistance = squaredDistanceList.at(0);
			AddNewMatchToCorrespondenceMap(correspondence);
			}
		}	
	}

void NeighbourSinglePointAverage::ComputeReplacementPoints()
	{
	firstReplacementMap.clear();
	secondReplacementMap.clear();

	//For each point of index x in first cloud, and for each point set of indeces y in second cloud, associate by the correspondenceMap,
	//we compute a single replacement point, given by the average between x and y.
	int numberOfCorrespondences = correspondenceMap.size();
	for(int correspondenceIndex = 0; correspondenceIndex < numberOfCorrespondences; correspondenceIndex++)
		{
		const Correspondence& correspondence = correspondenceMap.at(correspondenceIndex);
		pcl::PointXYZ firstPoint = firstCloud->points.at(correspondence.firstIndex);
		pcl::PointXYZ secondPoint = secondCloud->points.at(correspondence.secondIndex);
		pcl::PointXYZ replacementPoint = ComputeAveragePoint(firstPoint, secondPoint);
		firstReplacementMap[correspondence.firstIndex] = replacementPoint;
		secondReplacementMap[correspondence.secondIndex] = replacementPoint;
		}
	}

void NeighbourSinglePointAverage::AssemblePointCloud()
	{
	assembledCloud->points.resize(0);
	for(std::map<int, pcl::PointXYZ>::iterator replacementIterator = firstReplacementMap.begin(); replacementIterator != firstReplacementMap.end(); ++replacementIterator)
		{
		const pcl::PointXYZ& replacement = replacementIterator->second;
		assembledCloud->points.push_back( pcl::PointXYZ(replacement) );
		}

	AssembleLeftoverPoints(firstCloud, firstReplacementMap, parameters.ignoreUnmatchedPointsOnFirstCloud);
	AssembleLeftoverPoints(secondCloud, secondReplacementMap, parameters.ignoreUnmatchedPointsOnSecondCloud);
	}

void NeighbourSinglePointAverage::AssembleLeftoverPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::map<int, pcl::PointXYZ >& replacementMap, bool ignoreCloseUmantchedPoints)
	{
	//We compute replaced cloud as the point cloud containing all points in cloud that appear in replacement Map.
	pcl::PointCloud<pcl::PointXYZ>::Ptr replacedCloud( new pcl::PointCloud<pcl::PointXYZ> );

	//This will keep the correspondence between index in cloud and the matching point in replacedCloud.
	std::vector<int> replacedCloudToCloudIndexList;

	int numberOfPoints = cloud->points.size();
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		std::map<int, pcl::PointXYZ>::const_iterator replacement = replacementMap.find(pointIndex);
		if (replacement != replacementMap.end())
			{
			pcl::PointXYZ point = cloud->points.at(pointIndex);
			replacedCloud->points.push_back(point);
			replacedCloudToCloudIndexList.push_back(pointIndex);
			}
		}

	//If no points are replaced, than the clouds are not overlapping, we just put the whole input cloud into the fusion
	if (replacedCloud->points.size() == 0)
		{
		*assembledCloud += *cloud;
		return;
		}

	//This tree is meant for a neighbour search on replacedCloud
	pcl::KdTreeFLANN<pcl::PointXYZ> searchTree;
	searchTree.setInputCloud(replacedCloud);
	std::vector<int> indexList;
	std::vector<float> squaredDistanceList;

	//Every point P in cloud that does not appear in replacedCloud, will be translated by an amount D equal to the distance between P''-P' where P' is the closest point of replacedCloud to P,
	// and P'' is its replacement as defined in replacementMap if P' exists.
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		std::map<int, pcl::PointXYZ>::const_iterator replacement = replacementMap.find(pointIndex);
		if (replacement == replacementMap.end())
			{
			pcl::PointXYZ searchPoint = cloud->points.at(pointIndex);
			if (searchTree.nearestKSearch(searchPoint, 1, indexList, squaredDistanceList) > 0)
				{
				if (ignoreCloseUmantchedPoints && squaredDistanceList.at(0) < parameters.maxNeighbourDistance)
					{
					continue;
					}
				int closestIndex = indexList.at(0);
				int replacedPointIndex = replacedCloudToCloudIndexList.at(closestIndex);
				pcl::PointXYZ replacedPoint = cloud->points.at(replacedPointIndex);
				std::map<int, pcl::PointXYZ>::const_iterator replacementElement = replacementMap.find(replacedPointIndex);

				// The replacementElement should be valid because replacedPointIndex is contained in replacedCloudToCloudIndexList which contains only index of replaced points;
				ASSERT(replacementElement != replacementMap.end(), "NeighbourSinglePointAverage error, replacementElement should be valid");
				const pcl::PointXYZ& replacementPoint = replacementElement->second;

				pcl::PointXYZ newPoint = DisplacePointBySameDistance(searchPoint, replacedPoint, replacementPoint);
				assembledCloud->points.push_back(newPoint);
				}
			}
		}	
	}

pcl::PointXYZ NeighbourSinglePointAverage::ComputeAveragePoint(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::vector<int>& indexList)
	{
	int numberOfIndices = indexList.size();
	ASSERT(numberOfIndices > 0, "NeighbourSinglePointAverage error, cannot compute the average of zero points");

	pcl::PointXYZ averagePoint(0, 0, 0);
	for(int indexIndex = 0; indexIndex < numberOfIndices; indexIndex++)
		{
		int pointIndex = indexList.at(indexIndex);
		pcl::PointXYZ point = cloud->points.at(pointIndex);
		averagePoint.x += point.x;
		averagePoint.y += point.y;
		averagePoint.z += point.z;
		}
	averagePoint.x /= numberOfIndices;
	averagePoint.y /= numberOfIndices;
	averagePoint.z /= numberOfIndices;
	return averagePoint;
	}

pcl::PointXYZ NeighbourSinglePointAverage::ComputeAveragePoint(const pcl::PointXYZ& firstPoint, const pcl::PointXYZ& secondPoint)
	{
	pcl::PointXYZ averagePoint;
	averagePoint.x = (firstPoint.x + secondPoint.x) / 2;
	averagePoint.y = (firstPoint.y + secondPoint.y) / 2;
	averagePoint.z = (firstPoint.z + secondPoint.z) / 2;
	return averagePoint;
	}

pcl::PointXYZ NeighbourSinglePointAverage::DisplacePointBySameDistance(const pcl::PointXYZ& pointToDisplace, const pcl::PointXYZ& startDistanceReference, const pcl::PointXYZ& endDistanceReference)
	{
	pcl::PointXYZ displacedPoint;
	displacedPoint.x = pointToDisplace.x + (endDistanceReference.x - startDistanceReference.x);
	displacedPoint.y = pointToDisplace.y + (endDistanceReference.y - startDistanceReference.y);
	displacedPoint.z = pointToDisplace.z + (endDistanceReference.z - startDistanceReference.z);
	return displacedPoint;
	}

void NeighbourSinglePointAverage::PrepareOutAssembledPointCloud()
	{
	float squaredViewRadius = (inViewRadius*inViewRadius);
	int numberOfPoints = assembledCloud->points.size();
	ClearPoints(outAssembledPointCloud);

	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		pcl::PointXYZ point = assembledCloud->points.at(pointIndex);
		bool pointToAdd = (GetNumberOfPoints(outAssembledPointCloud) < PointCloudWrapper::MAX_CLOUD_SIZE);
		if (pointToAdd && parameters.useDistanceFilter)
			{
			float distanceX = GetXPosition(inViewCenter) - point.x;
			float distanceY = GetYPosition(inViewCenter) - point.y;
			float distanceZ = GetZPosition(inViewCenter) - point.z;
			float squaredDistance = distanceX*distanceX + distanceY*distanceY + distanceZ*distanceZ;
			pointToAdd = squaredDistance < squaredViewRadius;
			}
		if (pointToAdd)
			{
			AddPoint(outAssembledPointCloud, point.x, point.y, point.z);
			}
		}
	}

void NeighbourSinglePointAverage::AddNewMatchToCorrespondenceMap(Correspondence newCorrespondence)
	{
	int numberOfCorrespondences = correspondenceMap.size();
	for(int correspondenceIndex = 0; correspondenceIndex < numberOfCorrespondences; correspondenceIndex++)
		{
		Correspondence& correspondence = correspondenceMap.at(correspondenceIndex);
		bool conflictCorrespondence = (correspondence.firstIndex == newCorrespondence.firstIndex) || (correspondence.secondIndex == newCorrespondence.secondIndex);
		
		if(conflictCorrespondence)
			{
			bool betterCorrespondence = newCorrespondence.squaredDistance < correspondence.squaredDistance;
			if (betterCorrespondence)
				{
				correspondence = newCorrespondence;
				}
			return; // if we find a conflict, and the conflicting match is better, we just stop.
			}
		}
	//At this point we found no conflicts.
	correspondenceMap.push_back(newCorrespondence);
	}

void NeighbourSinglePointAverage::ValidateParameters()
{
	ASSERT( parameters.maxNeighbourDistance > 0, "NeighbourSinglePointAverage Configuration error, maxNeighbourDistance is not positive");
}

}
}
}

/** @} */
