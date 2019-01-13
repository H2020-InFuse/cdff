/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "IssDetector3D.hpp"

#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

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

IssDetector3D::IssDetector3D()
{
	parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<double>("GeneralParameters", "SalientRadius", parameters.salientRadius, DEFAULT_PARAMETERS.salientRadius);
	parametersHelper.AddParameter<double>("GeneralParameters", "NonMaximaSupressionRadius", parameters.nonMaximaSupressionRadius, DEFAULT_PARAMETERS.nonMaximaSupressionRadius);
	parametersHelper.AddParameter<double>("GeneralParameters", "NormalRadius", parameters.normalRadius, DEFAULT_PARAMETERS.normalRadius);
	parametersHelper.AddParameter<double>("GeneralParameters", "FirstThreshold", parameters.firstThreshold, DEFAULT_PARAMETERS.firstThreshold);
	parametersHelper.AddParameter<double>("GeneralParameters", "SecondThreshold", parameters.secondThreshold, DEFAULT_PARAMETERS.secondThreshold);
	parametersHelper.AddParameter<int>("GeneralParameters", "MinNumberOfNeighbours", parameters.minNumberOfNeighbours, DEFAULT_PARAMETERS.minNumberOfNeighbours);
	parametersHelper.AddParameter<float>("GeneralParameters", "AngleThreshold", parameters.angleThreshold, DEFAULT_PARAMETERS.angleThreshold);
	parametersHelper.AddParameter<int>("GeneralParameters", "NumberOfThreads", parameters.numberOfThreads, DEFAULT_PARAMETERS.numberOfThreads);
	parametersHelper.AddParameter<OutputFormat, OutputFormatHelper>("GeneralParameters", "OutputFormat", parameters.outputFormat, DEFAULT_PARAMETERS.outputFormat);

	configurationFilePath = "";
}

IssDetector3D::~IssDetector3D()
{
}

void IssDetector3D::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void IssDetector3D::process()
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
	pcl::PointIndicesConstPtr issPoints = ComputeIssPoints(inputPointCloud);

	// Write data to output port
	VisualPointFeatureVector3DConstPtr tmp = Convert(inputPointCloud, issPoints);
	Copy(*tmp, outFeatures);
	delete(tmp);
}

IssDetector3D::OutputFormatHelper::OutputFormatHelper(const std::string& parameterName, OutputFormat& boundVariable, const OutputFormat& defaultValue)
	: ParameterHelper(parameterName, boundVariable, defaultValue)
{
}

IssDetector3D::OutputFormat IssDetector3D::OutputFormatHelper::Convert(const std::string& outputFormat)
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

const IssDetector3D::IssOptionsSet IssDetector3D::DEFAULT_PARAMETERS
{
	/*.salientRadius =*/ 1e-4,
	/*.nonMaximaSupressionRadius =*/ 1e-4,
	/*.normalRadius =*/ 0.0,
	/*.firstThreshold =*/ 0.975,
	/*.secondThreshold =*/ 0.975,
	/*.minNumberOfNeighbours =*/ 5,
	/*.angleThreshold =*/ M_PI/2,
	/*.numberOfThreads =*/ 0,
	/*.outputFormat =*/ POSITIONS_OUTPUT
};

VisualPointFeatureVector3DConstPtr IssDetector3D::Convert(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud, const pcl::PointIndicesConstPtr indicesList)
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

pcl::PointIndicesConstPtr IssDetector3D::ComputeIssPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud)
{
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ, pcl::Normal> detector;
	detector.setSalientRadius(parameters.salientRadius);
	detector.setNonMaxRadius(parameters.nonMaximaSupressionRadius);
	detector.setNormalRadius(parameters.normalRadius);
	detector.setThreshold21 (parameters.firstThreshold);
	detector.setThreshold32 (parameters.secondThreshold);
	detector.setMinNeighbors  (parameters.minNumberOfNeighbours);
	detector.setAngleThreshold  (parameters.angleThreshold);
	detector.setNumberOfThreads(parameters.numberOfThreads);
	detector.setInputCloud(pointCloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	detector.compute(*keypoints);
	pcl::PointIndicesConstPtr issPoints = detector.getKeypointsIndices();

	return issPoints;
}

void IssDetector3D::ValidateParameters()
{
	ASSERT( parameters.salientRadius > 0, "IssDetector3D Configuration error, salientRadius is not positive");
	ASSERT( parameters.nonMaximaSupressionRadius > 0, "IssDetector3D Configuration error, nonMaximaSupressionRadius is not positive");
	ASSERT( parameters.normalRadius >= 0, "IssDetector3D Configuration error, normalRadius is negative");
	ASSERT( parameters.minNumberOfNeighbours >= 0, "IssDetector3D Configuration error, minNumberOfNeighbours is negative");
	ASSERT( parameters.numberOfThreads >= 0, "IssDetector3D Configuration error, number of threads is negative");
}

void IssDetector3D::ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud)
{
	for (size_t pointIndex = 0; pointIndex < pointCloud->points.size(); pointIndex++)
	{
		pcl::PointXYZ point = pointCloud->points.at(pointIndex);
		if (point.x != point.x || point.y != point.y || point.z != point.z)
		{
			ASSERT(false, "IssDetector3D Error: Invalid point in input point cloud");
		}
	}
}

}
}
}

/** @} */
