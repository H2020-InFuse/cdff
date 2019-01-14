/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "CornerDetector3D.hpp"

#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>

#include <stdlib.h>
#include <fstream>

using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;

namespace CDFF
{
namespace DFN
{
namespace FeaturesExtraction3D
{

CornerDetector3D::CornerDetector3D()
{
	parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<int>("GeneralParameters", "NumberOfNeighboursNormalEstimation", parameters.numberOfNeighboursNormalEstimation, DEFAULT_PARAMETERS.numberOfNeighboursNormalEstimation);
	parametersHelper.AddParameter<int>("GeneralParameters", "NumberOfNeighboursRegionGrowing", parameters.numberOfNeighboursRegionGrowing, DEFAULT_PARAMETERS.numberOfNeighboursRegionGrowing);
	parametersHelper.AddParameter<int>("GeneralParameters", "NumberOfNeightbourBorderSelection", parameters.numberOfNeightbourBorderSelection, DEFAULT_PARAMETERS.numberOfNeightbourBorderSelection);
	parametersHelper.AddParameter<float>("GeneralParameters", "SmoothnessThreshold", parameters.smoothnessThreshold, DEFAULT_PARAMETERS.smoothnessThreshold);
	parametersHelper.AddParameter<float>("GeneralParameters", "CurvatureThreshold", parameters.curvatureThreshold, DEFAULT_PARAMETERS.curvatureThreshold);
	parametersHelper.AddParameter<int>("GeneralParameters", "MinimumClusterSize", parameters.minimumClusterSize, DEFAULT_PARAMETERS.minimumClusterSize);
	parametersHelper.AddParameter<int>("GeneralParameters", "MaximumClusterSize", parameters.maximumClusterSize, DEFAULT_PARAMETERS.maximumClusterSize);
	parametersHelper.AddParameter<float>("GeneralParameters", "SearchRadiusBorderSelection", parameters.searchRadiusBorderSelection, DEFAULT_PARAMETERS.searchRadiusBorderSelection);
	parametersHelper.AddParameter<int>("GeneralParameters", "NumberOfHistogramSlots", parameters.numberOfHistogramSlots, DEFAULT_PARAMETERS.numberOfHistogramSlots);
	parametersHelper.AddParameter<int>("GeneralParameters", "MaximumNumberOfEmptySlots", parameters.maximumNumberOfEmptySlots, DEFAULT_PARAMETERS.maximumNumberOfEmptySlots);
	parametersHelper.AddParameter<OutputFormat, OutputFormatHelper>("GeneralParameters", "OutputFormat", parameters.outputFormat, DEFAULT_PARAMETERS.outputFormat);

	configurationFilePath = "";
}

CornerDetector3D::~CornerDetector3D()
{
}

void CornerDetector3D::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void CornerDetector3D::process()
{
	// Handle empty pointcloud
	if (GetNumberOfPoints(inPointcloud) == 0)
	{
		ClearPoints(outFeatures);
		return;
	}

	// Read data from input port
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputPointCloud =
		pointCloudToPclPointCloud.Convert(&inPointcloud);

	// Process data
	ValidateInputs(inputPointCloud);
	pcl::PointIndicesConstPtr corners = DetectCorners(inputPointCloud);

	// Write data to output port
	VisualPointFeatureVector3DConstPtr tmp = Convert(inputPointCloud, corners);
	Copy(*tmp, outFeatures);
	delete(tmp);
}

CornerDetector3D::OutputFormatHelper::OutputFormatHelper(const std::string& parameterName, OutputFormat& boundVariable, const OutputFormat& defaultValue)
	: ParameterHelper(parameterName, boundVariable, defaultValue)
{
}

CornerDetector3D::OutputFormat CornerDetector3D::OutputFormatHelper::Convert(const std::string& outputFormat)
{
	if (outputFormat == "Positions" || outputFormat == "0")
	{
		return POSITIONS_OUTPUT;
	}
	else if (outputFormat == "References" || outputFormat == "1")
	{
		return REFERENCES_OUTPUT;
	}
	ASSERT(false, "ShotDescriptor3d Error: unhandled output format");
	return POSITIONS_OUTPUT;
}

const CornerDetector3D::CornerOptionsSet CornerDetector3D::DEFAULT_PARAMETERS
{
	/*.numberOfNeighboursNormalEstimation =*/ 50,
	/*.numberOfNeighboursRegionGrowing =*/ 40,
	/*.numberOfNeightbourBorderSelection =*/ 15,
	/*.smoothnessThreshold =*/ 3.0 / 0.080,
	/*.curvatureThreshold =*/ 0.300,
	/*.minimumClusterSize =*/ 1,
	/*.maximumClusterSize =*/ 1000000,
	/*.searchRadiusBorderSelection =*/ 0.15,
	/*.numberOfHistogramSlots =*/ 10,
	/*.maximumNumberOfEmptySlots =*/ 3,
	/*.outputFormat =*/ POSITIONS_OUTPUT
};

VisualPointFeatureVector3DConstPtr CornerDetector3D::Convert(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud, const pcl::PointIndicesConstPtr indicesList)
{
	VisualPointFeatureVector3DPtr featuresVector = new VisualPointFeatureVector3D();
	ClearPoints(*featuresVector);
	for (unsigned pointIndex = 0; pointIndex < indicesList->indices.size() && pointIndex < MAX_FEATURE_3D_POINTS; pointIndex++)
	{
		if (parameters.outputFormat == POSITIONS_OUTPUT)
		{
			pcl::PointXYZ point = inputCloud->points.at(indicesList->indices.at(pointIndex));
			AddPoint(*featuresVector, point.x, point.y, point.z);
		}
		else if (parameters.outputFormat == REFERENCES_OUTPUT)
		{
			AddPoint(*featuresVector, indicesList->indices.at(pointIndex) );
		}
	}
	return featuresVector;
}

pcl::PointIndicesConstPtr CornerDetector3D::DetectCorners(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud)
{
	//Initialization of Core PCL computation objects
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

	//Estimating normals for the point cloud
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
	normalEstimator.setSearchMethod (tree);
	normalEstimator.setInputCloud (pointCloud);
	normalEstimator.setKSearch (parameters.numberOfNeighboursNormalEstimation);
	normalEstimator.compute (*normals);

	//Initializing Region Growing algorithm for cluster extraction based on normal similarity
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> segmentationEstimator;
	segmentationEstimator.setMinClusterSize (parameters.minimumClusterSize);
	segmentationEstimator.setMaxClusterSize (parameters.maximumClusterSize);
	segmentationEstimator.setSearchMethod (tree);
	segmentationEstimator.setNumberOfNeighbours (parameters.numberOfNeighboursRegionGrowing);
	segmentationEstimator.setInputCloud (pointCloud);
	segmentationEstimator.setInputNormals (normals);
	segmentationEstimator.setSmoothnessThreshold (parameters.smoothnessThreshold);
	segmentationEstimator.setCurvatureThreshold (parameters.curvatureThreshold);

	//extracting clusters
	std::vector <pcl::PointIndices> clusters;
	segmentationEstimator.extract (clusters);
	int numberOfClusters = clusters.size();

	//Creating the set of all points belonging to some cluster
	boost::shared_ptr<std::vector<int> > fullClusterIndices = boost::shared_ptr<std::vector<int> >( new std::vector<int> );
	for(int clusterIndex = 0; clusterIndex < numberOfClusters; clusterIndex++)
		{
		const pcl::PointIndices& cluster = clusters.at(clusterIndex);
		fullClusterIndices->insert(fullClusterIndices->end(), cluster.indices.begin(), cluster.indices.end());
		}

	//Creating a KD tree parse structure for the points in the cluster
	pcl::KdTreeFLANN<pcl::PointXYZ> fullKdtree;
	fullKdtree.setInputCloud(pointCloud, fullClusterIndices);

	//Creating a KD tree parse structure for the full point cloud
	pcl::KdTreeFLANN<pcl::PointXYZ> totalKdtree;
	totalKdtree.setInputCloud(pointCloud);

	//Initializing data structures for the following loop
	std::vector<int> indexList, fullIndexList, totalIndexList;
	std::vector<float> squaredDistanceList, fullSquaredDistanceList, totalSquaredDistanceList;
	pcl::PointIndicesPtr corners(new pcl::PointIndices);

	//This loop selects all those cluster points that are at the border of the cluster with other clusters, but not at the border of the input cloud
	for(int clusterIndex = 0; clusterIndex < numberOfClusters; clusterIndex++)
		{
		const pcl::PointIndices& cluster = clusters.at(clusterIndex);
		boost::shared_ptr< const std::vector<int> > indices = boost::shared_ptr<const std::vector<int> >( new std::vector<int>(cluster.indices) );
	
		//Creating a KD tree parse structure for the single cluster
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud (pointCloud, indices );

		int numberOfPoints = cluster.indices.size();
		for(int indexIndex = 0; indexIndex < numberOfPoints; indexIndex++)
			{
			int pointIndex = cluster.indices.at(indexIndex);
			const pcl::PointXYZ& searchPoint = pointCloud->points.at(pointIndex);

			//Evaluating neighborhood of single point in the total point cloud, in the total set of clusters and in the single cluster it belongs to
			kdtree.radiusSearch(searchPoint, parameters.searchRadiusBorderSelection, indexList, squaredDistanceList);
			fullKdtree.radiusSearch(searchPoint, parameters.searchRadiusBorderSelection, fullIndexList, fullSquaredDistanceList);
			totalKdtree.radiusSearch(searchPoint, parameters.searchRadiusBorderSelection, totalIndexList, totalSquaredDistanceList);

			//Accepting the point if it is at the border of the cluster with other clusters, but not at the border of the input cloud
			if (fullIndexList.size() > indexList.size() + parameters.numberOfNeightbourBorderSelection && PointIsInner(pointCloud, searchPoint, normals->points.at(pointIndex), totalIndexList) )
				{
				corners->indices.push_back(pointIndex);
				}
			}
		}

	return corners;
}

/* A point is cosidered Inner if it is not at the border of the total point cloud, i.e. its neighbourhood contains a point from the total point cloud in all directions */
bool CornerDetector3D::PointIsInner(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, const pcl::PointXYZ& originPoint, const pcl::Normal& normal, const std::vector<int>& neighbours)
	{
	//If the neighborhood is too small, this is an isolated point
	if (neighbours.size() < parameters.numberOfHistogramSlots - parameters.maximumNumberOfEmptySlots)
		{
		return false;
		}

	int numberOfNeighbours = neighbours.size();
	pcl::PointXYZ referenceAxisX;
	pcl::PointXYZ referenceAxisY;
	std::vector<int> histogram(parameters.numberOfHistogramSlots);
	for(int slot=0; slot<parameters.numberOfHistogramSlots; slot++)
		{
		histogram[slot] = 0;
		}
	
	//Computing the unit normal of the surface
	float normalNorm = std::sqrt(normal.normal_x*normal.normal_x + normal.normal_y*normal.normal_y + normal.normal_z*normal.normal_z);
	pcl::PointXYZ unitNormal(normal.normal_x / normalNorm, normal.normal_y / normalNorm, normal.normal_z / normalNorm);

	//This loop computes an histogram that counts the number of neighbours whose projection on the surface plane (normal to the UnitNormal) appears at a certain angle interval
	//ReferenceAxisX and ReferenceAxisY are used to compute the angle on the surface plane.
	for(int pointIndex = numberOfNeighbours-1; pointIndex >= 0; pointIndex--)
		{
		const pcl::PointXYZ point = pointCloud->points.at( neighbours.at(pointIndex) );
		pcl::PointXYZ distanceVector ( point.x - originPoint.x, point.y - originPoint.y, point.z - originPoint.z );

		float pointPlaneDistance = (distanceVector.x * unitNormal.x + distanceVector.y * unitNormal.y + distanceVector.z * unitNormal.z);
		pcl::PointXYZ projection ( point.x - pointPlaneDistance * unitNormal.x, point.y - pointPlaneDistance * unitNormal.y, point.z - pointPlaneDistance * unitNormal.z);

		//Computing the referenceAxisX in the direction of the point with index 0, referenceAxisY is othogonal to referenceAxisX
		if (pointIndex == 0)
			{
			referenceAxisX.x = projection.x - originPoint.x;
			referenceAxisX.y = projection.y - originPoint.y;
			referenceAxisX.z = projection.z - originPoint.z;
			float referenceAxisXNorm = std::sqrt(referenceAxisX.x*referenceAxisX.x + referenceAxisX.y*referenceAxisX.y + referenceAxisX.z*referenceAxisX.z);
			referenceAxisX.x /= referenceAxisXNorm;
			referenceAxisX.y /= referenceAxisXNorm;
			referenceAxisX.z /= referenceAxisXNorm;

			referenceAxisY.x = unitNormal.y * referenceAxisX.z - unitNormal.z * referenceAxisX.y;
			referenceAxisY.y = unitNormal.z * referenceAxisX.x - unitNormal.x * referenceAxisX.z;
			referenceAxisY.z = unitNormal.x * referenceAxisX.y - unitNormal.y * referenceAxisX.x;
			float referenceAxisYNorm = std::sqrt(referenceAxisY.x*referenceAxisY.x + referenceAxisY.y*referenceAxisY.y + referenceAxisY.z*referenceAxisY.z);
			referenceAxisY.x /= referenceAxisYNorm;
			referenceAxisY.y /= referenceAxisYNorm;
			referenceAxisY.z /= referenceAxisYNorm;
			}

		//Computing projections and angle
		float projectionX = distanceVector.x * referenceAxisX.x + distanceVector.y * referenceAxisX.y + distanceVector.z * referenceAxisX.z;
		float projectionY = distanceVector.x * referenceAxisY.x + distanceVector.y * referenceAxisY.y + distanceVector.z * referenceAxisY.z;
		float angle = std::atan2(projectionY, projectionX);

		//Updating Histograms
		for(int slot=0; slot< parameters.numberOfHistogramSlots; slot++)
			{
			float bottomLimit = static_cast<float>(slot) * 2*M_PI/static_cast<float>(parameters.numberOfHistogramSlots);
			float upperLimit = static_cast<float>(slot+1) * 2*M_PI/static_cast<float>(parameters.numberOfHistogramSlots);
			if ( (angle >= bottomLimit && angle <= upperLimit) || slot == parameters.numberOfHistogramSlots-1)
				{
				histogram.at(slot) = histogram.at(slot) + 1;
				break;
				}
			}	
		}

	//We count the number of empty histogram slots
	int emptySlotCounter = 0;
	for(int slot=0; slot<parameters.numberOfHistogramSlots; slot++)
		{
		emptySlotCounter = (histogram.at(slot) == 0 ? emptySlotCounter+1 : emptySlotCounter);
		}

	//We consider the point to be internal to the full point cloud if the number of histogram slots (i.e. angle intervals on the surface plane) that contains no neighbour is small enough
	return emptySlotCounter <= parameters.maximumNumberOfEmptySlots;
	}

void CornerDetector3D::ValidateParameters()
{
	ASSERT( parameters.numberOfNeighboursNormalEstimation > 0, "CornerDetector3D Configuration error, numberOfNeighboursNormalEstimation is not positive");
	ASSERT( parameters.numberOfNeighboursRegionGrowing > 0, "CornerDetector3D Configuration error, numberOfNeighboursRegionGrowing is not positive");
	ASSERT( parameters.numberOfNeightbourBorderSelection > 0, "CornerDetector3D Configuration error, numberOfNeightbourBorderSelection is not positive");
	ASSERT( parameters.smoothnessThreshold > 0, "CornerDetector3D Configuration error, smoothnessThreshold is not positive");
	ASSERT( parameters.curvatureThreshold > 0, "CornerDetector3D Configuration error, curvatureThreshold is not positive");
	ASSERT( parameters.minimumClusterSize > 0, "CornerDetector3D Configuration error, minimumClusterSize is not positive");
	ASSERT( parameters.maximumClusterSize > parameters.minimumClusterSize, "CornerDetector3D Configuration error, maximumClusterSize is not greater than minimumClusterSize");
	ASSERT( parameters.searchRadiusBorderSelection > 0, "CornerDetector3D Configuration error, minimumClusterSize is not positive");
	ASSERT( parameters.numberOfHistogramSlots > 0, "CornerDetector3D Configuration error, numberOfHistogramSlots is not positive");
	ASSERT( parameters.maximumNumberOfEmptySlots <= parameters.numberOfHistogramSlots, "CornerDetector3D Configuration error, maximumNumberOfEmptySlots is larger than numberOfHistogramSlots");
}

void CornerDetector3D::ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud)
{
	for (size_t pointIndex = 0; pointIndex < pointCloud->points.size(); pointIndex++)
	{
		pcl::PointXYZ point = pointCloud->points.at(pointIndex);
		if (point.x != point.x || point.y != point.y || point.z != point.z)
		{
			ASSERT(false, "CornerDetector3D Error: Invalid point in input point cloud");
		}
	}
}

}
}
}

/** @} */
