/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "PfhDescriptor3D.hpp"

#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/PointCloudToPclNormalsCloudConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <stdlib.h>
#include <fstream>

using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;

namespace CDFF
{
namespace DFN
{
namespace FeaturesDescription3D
{

PfhDescriptor3D::PfhDescriptor3D()
{
	parameters = DEFAULT_PARAMETERS;

	parametersHelper.AddParameter<double>(
		"GeneralParameters", "SearchRadius",
		parameters.baseOptions.searchRadius,
		DEFAULT_PARAMETERS.baseOptions.searchRadius);
	parametersHelper.AddParameter<OutputFormat, OutputFormatHelper>(
		"GeneralParameters", "OutputFormat",
		parameters.baseOptions.outputFormat,
		DEFAULT_PARAMETERS.baseOptions.outputFormat);
	parametersHelper.AddParameter<bool>(
		"GeneralParameters", "EnableNormalsEstimation",
		parameters.baseOptions.enableNormalsEstimation,
		DEFAULT_PARAMETERS.baseOptions.enableNormalsEstimation);
	parametersHelper.AddParameter<bool>(
		"GeneralParameters", "ForceNormalsEstimation",
		parameters.baseOptions.forceNormalsEstimation,
		DEFAULT_PARAMETERS.baseOptions.forceNormalsEstimation);

	parametersHelper.AddParameter<double>(
		"NormalEstimationParameters", "SearchRadius",
		parameters.normalEstimationOptions.searchRadius,
		DEFAULT_PARAMETERS.normalEstimationOptions.searchRadius);
	parametersHelper.AddParameter<int>(
		"NormalEstimationParameters", "NeighboursSetSize",
		parameters.normalEstimationOptions.neighboursSetSize,
		DEFAULT_PARAMETERS.normalEstimationOptions.neighboursSetSize);

	configurationFilePath = "";
}

PfhDescriptor3D::~PfhDescriptor3D()
{
}

void PfhDescriptor3D::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void PfhDescriptor3D::process()
{
	// Handle empty keypoint vector
	if (GetNumberOfPoints(inFeatures) == 0 || GetNumberOfPoints(inPointcloud) == 0)
	{
		ClearPoints(outFeatures);
		return;
	}

	// Read data from input ports
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputPointCloud =
		pointCloudToPclPointCloud.Convert(&inPointcloud);
	pcl::IndicesConstPtr indicesList = Convert(&inFeatures);
	pcl::PointCloud<pcl::Normal>::ConstPtr inputNormalsCloud =
		pointCloudToPclNormalsCloud.Convert(&inNormals);

	// Process data
	ValidateMandatoryInputs(inputPointCloud, indicesList);
	pcl::PointCloud<pcl::PFHSignature125>::ConstPtr pfhPointCloud = ComputePfhDescriptors(inputPointCloud, indicesList, inputNormalsCloud);

	// Write data to output port
	VisualPointFeatureVector3DConstPtr tmp = Convert(inputPointCloud, indicesList, pfhPointCloud);
	Copy(*tmp, outFeatures);
	delete(tmp);
}

PfhDescriptor3D::OutputFormatHelper::OutputFormatHelper(const std::string& parameterName, OutputFormat& boundVariable, const OutputFormat& defaultValue) :
	ParameterHelper(parameterName, boundVariable, defaultValue)
{
}

PfhDescriptor3D::OutputFormat PfhDescriptor3D::OutputFormatHelper::Convert(const std::string& outputFormat)
{
	if (outputFormat == "Positions" || outputFormat == "0")
	{
		return POSITIONS_OUTPUT;
	}
	else if (outputFormat == "References" || outputFormat == "1")
	{
		return REFERENCES_OUTPUT;
	}
	ASSERT(false, "PfhDescriptor3D Error: unhandled output format");
	return POSITIONS_OUTPUT;
}

const PfhDescriptor3D::PfhOptionsSet PfhDescriptor3D::DEFAULT_PARAMETERS
{
	//.baseOptions =
	{
		/*.searchRadius =*/ 0.01,
		/*.outputFormat =*/ POSITIONS_OUTPUT,
		/*.enableNormalsEstimation =*/ true,
		/*.forceNormalsEstimation =*/ true
	},
	//.normalEstimationOptions =
	{
		/*.searchRadius =*/ 0.00,
		/*.neighboursSetSize =*/ 10
	}
};

pcl::IndicesConstPtr PfhDescriptor3D::Convert( const VisualPointFeatureVector3DConstPtr featuresVector)
{
	ASSERT(GetNumberOfPoints(*featuresVector) == 0 || GetVectorType(*featuresVector) == ALL_REFERENCES_VECTOR, "PfhDescriptor3D: input features set does not contain all reference-defined points");
	pcl::IndicesPtr indicesList = boost::make_shared<std::vector<int> >();

	for (int pointIndex = 0; pointIndex < GetNumberOfPoints(*featuresVector); pointIndex++)
	{
		indicesList->push_back( GetReferenceIndex(*featuresVector, pointIndex) );
	}

	return indicesList;
}

VisualPointFeatureVector3DConstPtr PfhDescriptor3D::Convert(
	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud,
	const pcl::IndicesConstPtr indicesList,
	const pcl::PointCloud<pcl::PFHSignature125>::ConstPtr pfhPointCloud)
{
	static const float LARGE_NUMBER_THAT_REPLACES_NAN = 1000;
	static const unsigned PFH_DESCRIPTOR_SIZE = 125;
	VisualPointFeatureVector3DPtr featuresVector = new VisualPointFeatureVector3D();
	ClearPoints(*featuresVector);
	bool computedNanFeature = false;
	for (unsigned pointIndex = 0; pointIndex < indicesList->size() && pointIndex < MAX_FEATURE_3D_POINTS; pointIndex++)
	{
		if (parameters.baseOptions.outputFormat == POSITIONS_OUTPUT)
		{
			pcl::PointXYZ point = inputCloud->points.at(indicesList->at(pointIndex));
			AddPoint(*featuresVector, point.x, point.y, point.z);
		}
		else if (parameters.baseOptions.outputFormat == REFERENCES_OUTPUT)
		{
			AddPoint(*featuresVector, indicesList->at(pointIndex));
		}

		pcl::PFHSignature125 feature = pfhPointCloud->points.at(pointIndex);
		for (unsigned componentIndex = 0; componentIndex < PFH_DESCRIPTOR_SIZE; componentIndex++)
		{
			if (feature.histogram[componentIndex] != feature.histogram[componentIndex])
			{
				computedNanFeature = true;
				feature.histogram[componentIndex] = LARGE_NUMBER_THAT_REPLACES_NAN;
			}
			AddDescriptorComponent(*featuresVector, pointIndex, feature.histogram[componentIndex]);
		}
	}
	if (computedNanFeature)
	{
		DEBUG_PRINT_TO_LOG("PFH 3D problem: At least one of the computed features is NaN", "");
	}
	return featuresVector;
}

pcl::PointCloud<pcl::PFHSignature125>::ConstPtr PfhDescriptor3D::ComputePfhDescriptors(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud,
	pcl::IndicesConstPtr indicesList,
	pcl::PointCloud<pcl::Normal>::ConstPtr optionalNormalsCloud)
{
	// Setup k-d tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();

	// Estimate surface normals, or use provided estimates
	pcl::PointCloud<pcl::Normal>::ConstPtr normalsCloud;

	if (parameters.baseOptions.forceNormalsEstimation == true)
	{
		// Compute estimates of the surface normals
		// regardless of the provided normals
		normalsCloud = EstimateNormals(pointCloud, kdTree);
	}
	else
	{
		// Check that the provided normals aren't grossly wrong
		bool validNormalsCloud = IsNormalsCloudValid(pointCloud, optionalNormalsCloud);

		if (parameters.baseOptions.enableNormalsEstimation == false)
		{
			// Use provided normals, unless they're grossly wrong,
			// in which case abort or throw exception
			ASSERT(validNormalsCloud,
				"PfhDescriptor3D Error: The provided normals are not usable");
			normalsCloud = optionalNormalsCloud;
		}
		else
		{
			// Use provided normals, unless they're grossly wrong,
			// in which case compute better estimates
			if (validNormalsCloud == true)
			{
				normalsCloud = optionalNormalsCloud;
			}
			else
			{
				normalsCloud = EstimateNormals(pointCloud, kdTree);
			}
		}
	}

	// Setup PCL's PFH algorithm
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setSearchMethod(kdTree);
	pfh.setInputCloud(pointCloud);
	pfh.setInputNormals(normalsCloud);
	pfh.setRadiusSearch(parameters.baseOptions.searchRadius);

	// Setup output
	pcl::PointCloud<pcl::PFHSignature125>::Ptr featuresCloud = boost::make_shared<pcl::PointCloud<pcl::PFHSignature125> >();

	// Compute PFH descriptors
	pfh.compute(*featuresCloud);

	// Sanity check
	ASSERT(featuresCloud->points.size() == indicesList->size(),
		"PfhDescriptor3D Error: the number of keypoints and the number of indices don't match");

	return featuresCloud;
}

pcl::PointCloud<pcl::Normal>::ConstPtr PfhDescriptor3D::EstimateNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud (pointCloud);
	normalEstimation.setSearchMethod (kdTree);
	if (parameters.normalEstimationOptions.searchRadius > 0)
	{
		normalEstimation.setRadiusSearch (parameters.normalEstimationOptions.searchRadius);
	}
	else
	{
		normalEstimation.setKSearch (parameters.normalEstimationOptions.neighboursSetSize);
	}

	pcl::PointCloud<pcl::Normal>::Ptr normalsCloud (new pcl::PointCloud<pcl::Normal>);
	normalEstimation.compute(*normalsCloud);

	return normalsCloud;
}

void PfhDescriptor3D::ValidateParameters()
{
	ASSERT(parameters.baseOptions.searchRadius >= 0,
		"PfhDescriptor3D Configuration error, search radius is negative");

	ASSERT(!parameters.baseOptions.forceNormalsEstimation || parameters.baseOptions.enableNormalsEstimation,
		"PfhDescriptor3D error, you cannot disable and force surface normal estimation at the same time");

	if (parameters.baseOptions.enableNormalsEstimation)
	{
		ASSERT(parameters.normalEstimationOptions.searchRadius >= 0,
			"PfhDescriptor3D Configuration error, normal estimation search radius is negative");
		ASSERT(parameters.normalEstimationOptions.neighboursSetSize >= 0,
			"PfhDescriptor3D Configuration error, neighbours set size is negative");
		if (parameters.normalEstimationOptions.searchRadius > 0)
		{
			ASSERT(parameters.normalEstimationOptions.neighboursSetSize == 0,
				"PfhDescriptor3D error, only one between normal estimation search radius and neighbours set size can be defined");
			ASSERT(parameters.normalEstimationOptions.searchRadius < parameters.baseOptions.searchRadius,
				"PfhDescriptor3D error, search radius for normal estimation should be smaller than search radius for descriptor computation");
		}
		else
		{
			ASSERT(parameters.normalEstimationOptions.neighboursSetSize > 0,
				"PfhDescriptor3D error, only one between normal estimation search radius and neighbours set size can be defined");
		}
	}
}

void PfhDescriptor3D::ValidateMandatoryInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::IndicesConstPtr indicesList)
{
	ASSERT(indicesList->size() <= pointCloud->points.size(), "PfhDescriptor3D Error: There are more keypoints than points in the pointcloud");

	for (size_t pointIndex = 0; pointIndex < pointCloud->points.size(); pointIndex++)
	{
		pcl::PointXYZ point = pointCloud->points.at(pointIndex);
		if (point.x != point.x || point.y != point.y || point.z != point.z)
		{
			ASSERT(false, "PfhDescriptor3D Error: There is an invalid point in the pointcloud");
		}
	}
}

bool PfhDescriptor3D::IsNormalsCloudValid(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::PointCloud<pcl::Normal>::ConstPtr normalsCloud)
{
	if (pointCloud->points.size() != normalsCloud->points.size())
	{
		ASSERT(parameters.baseOptions.enableNormalsEstimation, "PfhDescriptor3D Error: The number of points and the number of normals don't match");
		return false;
	}

	for (size_t pointIndex = 0; pointIndex < normalsCloud->points.size(); pointIndex++)
	{
		pcl::Normal normal = normalsCloud->points.at(pointIndex);
		if (normal.normal_x != normal.normal_x || normal.normal_y != normal.normal_y || normal.normal_z != normal.normal_z)
		{
			ASSERT(parameters.baseOptions.enableNormalsEstimation, "PfhDescriptor3D Error: There is an invalid normal to the pointcloud surface");
			return false;
		}
	}

	return true;
}

}
}
}

/** @} */
