/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "BestDescriptorMatch.hpp"

#include <Converters/EigenTransformToTransform3DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/registration/icp.h>
#include <pcl/common/geometry.h>
#include <yaml-cpp/yaml.h>

//#include <Visualizers/PCLVisualizer.hpp>

using namespace Converters;
using namespace PoseWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace SupportTypes;

namespace CDFF
{
namespace DFN
{
namespace FeaturesMatching3D
{

BestDescriptorMatch::BestDescriptorMatch()
{
	parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<double>(
		"GeneralParameters", "MaxCorrespondenceDistance",
		parameters.maxCorrespondenceDistance, DEFAULT_PARAMETERS.maxCorrespondenceDistance);

	configurationFilePath = "";
}

BestDescriptorMatch::~BestDescriptorMatch()
{
}

void BestDescriptorMatch::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void BestDescriptorMatch::process()
{
	// Handle empty keypoint vectors
	if (GetNumberOfPoints(inSourceFeatures) == 0 || GetNumberOfPoints(inSinkFeatures) == 0)
	{
		outSuccess = false;
		Reset(outTransform);
		return;
	}

	// Process data
	pcl::PointCloud<pcl::PointXYZ>::Ptr bestMatchSourceCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<pcl::PointXYZ>::Ptr bestMatchSinkCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	ComputeBestMatches(bestMatchSourceCloud, bestMatchSinkCloud);

	if (bestMatchSourceCloud->points.size() == 0)
		{
		outSuccess = false;
		return;
		}

	Pose3DConstPtr tmp = ComputeTransform(bestMatchSourceCloud, bestMatchSinkCloud);

	// Write data to output port
	outSuccess = true;
	Copy(*tmp, outTransform);
	delete tmp;
}

const BestDescriptorMatch::BestDescriptorMatchOptionsSet BestDescriptorMatch::DEFAULT_PARAMETERS =
{
	/*.maxCorrespondenceDistance =*/ 0.05
};

/*This method computes the point clouds containing the best matches among the points in the input clouds. 
Output: For each index i, bestMatchSourceCloud(i) is a point in sourceCloud that is the best match of bestMatchSinkCloud(i) among the points in sinkCloud */
void BestDescriptorMatch::ComputeBestMatches(pcl::PointCloud<pcl::PointXYZ>::Ptr bestMatchSourceCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr bestMatchSinkCloud)
	{
	int numberOfSourcePoints = GetNumberOfPoints(inSourceFeatures); 
	int numberOfSinkPoints = GetNumberOfPoints(inSinkFeatures);
	std::vector<float> minimumDistances(numberOfSourcePoints);
	std::vector<int> minimumSinkIndex(numberOfSourcePoints);

	// For each source point we compute the sink point whose feature distance from the source point is the minimum among all sink points
	for(int sourceIndex = 0; sourceIndex < numberOfSourcePoints; sourceIndex++)
		{
		float& minimumDistance = minimumDistances.at(sourceIndex);
		minimumDistance = std::numeric_limits<float>::infinity();

		for(int sinkIndex = 0; sinkIndex < numberOfSinkPoints; sinkIndex++)
			{
			float descriptorDistance = ComputeDistance(sourceIndex, sinkIndex);
			if (minimumDistance > descriptorDistance)
				{
				minimumDistance = descriptorDistance;
				minimumSinkIndex.at(sourceIndex) = sinkIndex;
				}
			}
		}

	//For each source point, we test whether the minimum distance computed above is belowe the maxCorrespondenceDistance parameters. If yes, a best match is recorded between
	//the source point and the minimum distance sink point associated to it.
	for(int sourceIndex = 0; sourceIndex < numberOfSourcePoints; sourceIndex++)
		{
		if ( minimumDistances.at(sourceIndex) < parameters.maxCorrespondenceDistance )
			{
			pcl::PointXYZ sourcePoint(GetXCoordinate(inSourceFeatures, sourceIndex),  GetYCoordinate(inSourceFeatures, sourceIndex), GetZCoordinate(inSourceFeatures, sourceIndex));
			int sinkIndex = minimumSinkIndex.at(sourceIndex);
			pcl::PointXYZ sinkPoint(GetXCoordinate(inSinkFeatures, sinkIndex),  GetYCoordinate(inSinkFeatures, sinkIndex), GetZCoordinate(inSinkFeatures, sinkIndex));

			bestMatchSourceCloud->points.push_back(sourcePoint);
			bestMatchSinkCloud->points.push_back(sinkPoint);
			}
		}
	}

Pose3DConstPtr BestDescriptorMatch::ComputeTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr sinkCloud)
{
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float> registration;

	Eigen::Matrix4f eigenTransform;
	registration.estimateRigidTransformation(*sourceCloud, *sinkCloud, eigenTransform);

	return EigenTransformToTransform3DConverter().Convert(eigenTransform);
}

float BestDescriptorMatch::ComputeDistance(int sourceIndex, int sinkIndex)
	{
	float squaredDistance = 0;
	int numberOfFeatureComponents = GetNumberOfDescriptorComponents(inSourceFeatures, sourceIndex);
	ASSERT( numberOfFeatureComponents == GetNumberOfDescriptorComponents(inSinkFeatures, sinkIndex), "BestDescriptorMatch error, mismatch size between source and sink descriptors");
	for(int componentIndex = 0; componentIndex < numberOfFeatureComponents; componentIndex++)
		{
		float componentDistance = GetDescriptorComponent(inSourceFeatures, sourceIndex, componentIndex) - GetDescriptorComponent(inSinkFeatures, sinkIndex, componentIndex);
		squaredDistance += componentDistance*componentDistance;
		}
	return std::sqrt(squaredDistance);
	}

Pose3DConstPtr BestDescriptorMatch::Convert(Eigen::Matrix4f eigenTransform)
{
	Pose3DPtr transform = new Pose3D;

	Eigen::Matrix3f eigenRotationMatrix = eigenTransform.block(0,0,3,3);
	Eigen::Quaternionf eigenRotation(eigenRotationMatrix);

	SetPosition(*transform, eigenTransform(0, 3), eigenTransform(1, 3), eigenTransform(2, 3) );
	SetOrientation(*transform, eigenRotation.x(), eigenRotation.y(), eigenRotation.z(), eigenRotation.w());

	return transform;
}

void BestDescriptorMatch::ValidateParameters()
{
	ASSERT(parameters.maxCorrespondenceDistance >= 0, "BestDescriptorMatch Configuration error, Max Correspondence Distance is negative");
}

void BestDescriptorMatch::ValidateInputs()
{
	ValidateCloud(inSourceFeatures);
	ValidateCloud(inSinkFeatures);
}

void BestDescriptorMatch::ValidateCloud(const VisualPointFeatureVector3D& features)
{
	int numberOfPoints = GetNumberOfPoints(features);
	if (numberOfPoints == 0)
		{
		return;
		}

	int baseDescriptorSize = GetNumberOfDescriptorComponents(features, 0);
	for (unsigned pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
	{
		float x = GetXCoordinate(features, pointIndex);
		float y = GetYCoordinate(features, pointIndex);
		float z = GetZCoordinate(features, pointIndex);
		ASSERT(x == x, "BestDescriptorMatch Error, cloud contains a NaN point");
		ASSERT(y == y, "BestDescriptorMatch Error, cloud contains a NaN point");
		ASSERT(z == z, "BestDescriptorMatch Error, cloud contains a NaN point");

		int descriptorSize = GetNumberOfDescriptorComponents(features, pointIndex);
		ASSERT(descriptorSize == baseDescriptorSize, "BestDescriptorMatch Error, Descriptor sizes mismatch");

		for (unsigned componentIndex = 0; componentIndex < descriptorSize; componentIndex++)
		{
			float component = GetDescriptorComponent(features, pointIndex, componentIndex);
			ASSERT(component == component, "BestDescriptorMatch Error, feature cloud contains a NaN feature");
		}
	}
}

}
}
}

/** @} */
