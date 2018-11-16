/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "MatcherAssembly.hpp"

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

MatcherAssembly::MatcherAssembly()
{
	parametersHelper.AddParameter<float>("GeneralParameters", "MaximumSamplingDensity", parameters.maximumSamplingDensity, DEFAULT_PARAMETERS.maximumSamplingDensity);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseIncrementalMode", parameters.useIncrementalMode, DEFAULT_PARAMETERS.useIncrementalMode);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseDistanceFilter", parameters.useDistanceFilter, DEFAULT_PARAMETERS.useDistanceFilter);

	parametersHelper.AddParameter<int>("SurfaceNormalPrefilter", "NumberOfNeighbours", 
		parameters.samplingSurfaceNormalPrefilter.numberOfNeighbours, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.numberOfNeighbours);
	parametersHelper.AddParameter<float>("SurfaceNormalPrefilter", "ApproximationEpsilon", 
		parameters.samplingSurfaceNormalPrefilter.approximationEpsilon, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.approximationEpsilon);
	parametersHelper.AddParameter<bool>("SurfaceNormalPrefilter", "KeepNormals", 
		parameters.samplingSurfaceNormalPrefilter.keepNormals, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.keepNormals);
	parametersHelper.AddParameter<bool>("SurfaceNormalPrefilter", "KeepDensities", 
		parameters.samplingSurfaceNormalPrefilter.keepDensities, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.keepDensities);
	parametersHelper.AddParameter<bool>("SurfaceNormalPrefilter", "KeepEigenValues", 
		parameters.samplingSurfaceNormalPrefilter.keepEigenValues, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.keepEigenValues);
	parametersHelper.AddParameter<bool>("SurfaceNormalPrefilter", "KeepEigenVectors", 
		parameters.samplingSurfaceNormalPrefilter.keepEigenVectors, DEFAULT_PARAMETERS.samplingSurfaceNormalPrefilter.keepEigenVectors);

	configurationFilePath = "";

	SetPosition(inViewCenter, 0, 0, 0);
	inViewRadius = 100;
}

MatcherAssembly::~MatcherAssembly()
{
}

void MatcherAssembly::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();

	{
	PointMatcherSupport::Parametrizable::Parameters configuration;
	configuration["knn"] = std::to_string(parameters.samplingSurfaceNormalPrefilter.numberOfNeighbours);
	configuration["epsilon"] = std::to_string(parameters.samplingSurfaceNormalPrefilter.approximationEpsilon);
	configuration["keepNormals"] = (parameters.samplingSurfaceNormalPrefilter.keepNormals ? "1" : "0");
	configuration["keepDensities"] = (parameters.samplingSurfaceNormalPrefilter.keepDensities ? "1" : "0");
	configuration["keepEigenValues"] = (parameters.samplingSurfaceNormalPrefilter.keepEigenValues ? "1" : "0");
	configuration["keepEigenVectors"] = (parameters.samplingSurfaceNormalPrefilter.keepEigenVectors ? "1" : "0");
	samplingSurfaceNormalPrefilter = PointMatcher<float>::get().DataPointsFilterRegistrar.create("SurfaceNormalDataPointsFilter", configuration);
	}

	{
	PointMatcherSupport::Parametrizable::Parameters configuration;
	configuration["maxDensity"] = std::to_string(parameters.maximumSamplingDensity);
	maxDensityFilter = PointMatcher<float>::get().DataPointsFilterRegistrar.create("MaxDensityDataPointsFilter", configuration);
	}
}

void MatcherAssembly::process()
{
	if (!parameters.useIncrementalMode)
		{
		PointMatcher<float>::DataPoints firstCloud = ConvertToDataPoints(inFirstPointCloud);
		PointMatcher<float>::DataPoints secondCloud = ConvertToDataPoints(inSecondPointCloud);
		AssemblePointCloud(firstCloud, secondCloud);
		}
	else
		{
		PointMatcher<float>::DataPoints secondCloud = ConvertToDataPoints(inFirstPointCloud);
		AssemblePointCloud(secondCloud);
		}

	PrepareOutAssembledPointCloud();
}

const MatcherAssembly::MatcherAssemblyOptionsSet MatcherAssembly::DEFAULT_PARAMETERS
{
	/*.maximumSamplingDensity =*/
		{
		/*.numberOfNeighbours =*/ 10,
		/*.approximationEpsilon =*/ 5,
		/*.KeepNormals =*/ false,
		/*.KeepDensities =*/ true,
		/*.keepEigenValues =*/ false,
		/*.keepEigenVectors =*/ false
		},
	/*.maximumSamplingDensity =*/ 30,
	/*.useIncrementalMode =*/ false,
	/*.useDistanceFilter =*/ false
};

void MatcherAssembly::AssemblePointCloud(const PointMatcher<float>::DataPoints& cloud)
	{
	if (assembledCloud.getNbPoints() == 0)
		{
		assembledCloud = cloud;
		}
	else
		{
		assembledCloud.concatenate(cloud);
		}
	if (assembledCloud.getNbPoints() > parameters.samplingSurfaceNormalPrefilter.numberOfNeighbours)
		{
		assembledCloud = samplingSurfaceNormalPrefilter->filter(assembledCloud);
		assembledCloud = maxDensityFilter->filter(assembledCloud);
		}
	}

void MatcherAssembly::AssemblePointCloud(const PointMatcher<float>::DataPoints& cloud1, const PointMatcher<float>::DataPoints& cloud2)
	{
	assembledCloud = cloud1;
	AssemblePointCloud(cloud2);
	}

void MatcherAssembly::PrepareOutAssembledPointCloud()
	{
	float squaredViewRadius = (inViewRadius*inViewRadius);
	int numberOfPoints = assembledCloud.getNbPoints();
	ClearPoints(outAssembledPointCloud);

	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		float xw = assembledCloud.getFeatureRowViewByName("x", 0)(0, pointIndex);
		float yw = assembledCloud.getFeatureRowViewByName("y", 0)(0, pointIndex);
		float zw = assembledCloud.getFeatureRowViewByName("z", 0)(0, pointIndex);
		float w = assembledCloud.getFeatureRowViewByName("w", 0)(0, pointIndex);
		if ( w < 1e-6 )
			{
			continue;
			}
		float x = xw/w;
		float y = yw/w;
		float z = zw/w;

		bool pointToAdd = (GetNumberOfPoints(outAssembledPointCloud) < PointCloudWrapper::MAX_CLOUD_SIZE);
		if (pointToAdd && parameters.useDistanceFilter)
			{
			float distanceX = GetXPosition(inViewCenter) - x;
			float distanceY = GetYPosition(inViewCenter) - y;
			float distanceZ = GetZPosition(inViewCenter) - z;
			float squaredDistance = distanceX*distanceX + distanceY*distanceY + distanceZ*distanceZ;
			pointToAdd = squaredDistance < squaredViewRadius;
			}
		if (pointToAdd)
			{
			AddPoint(outAssembledPointCloud, x, y, z);
			}
		}
	}

void MatcherAssembly::ValidateParameters()
{
	ASSERT( parameters.maximumSamplingDensity > 0, "MatcherAssembly Configuration error, maximumSamplingDensity is not positive");
	ASSERT( parameters.samplingSurfaceNormalPrefilter.numberOfNeighbours > 0, "MatcherAssembly Configuration error, numberOfNeighbours is not positive");
	ASSERT( parameters.samplingSurfaceNormalPrefilter.approximationEpsilon >= 0, "MatcherAssembly Configuration error, numberOfNeighbours is negative");
}

PointMatcher<float>::DataPoints MatcherAssembly::ConvertToDataPoints(const PointCloudWrapper::PointCloud& cloud)
	{
	int numberOfPoints = GetNumberOfPoints(cloud);
	PointMatcher<float>::Matrix cloudMatrix(4, numberOfPoints);
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		cloudMatrix(0, pointIndex) = GetXCoordinate(cloud, pointIndex);
		cloudMatrix(1, pointIndex) = GetYCoordinate(cloud, pointIndex);
		cloudMatrix(2, pointIndex) = GetZCoordinate(cloud, pointIndex);
		cloudMatrix(3, pointIndex) = 1;
		}

	PointMatcher<float>::DataPoints::Labels labels;
	labels.push_back( PointMatcher<float>::DataPoints::Label("x", 1)); 
	labels.push_back( PointMatcher<float>::DataPoints::Label("y", 1)); 
	labels.push_back( PointMatcher<float>::DataPoints::Label("z", 1)); 
	labels.push_back( PointMatcher<float>::DataPoints::Label("w", 1)); 

	PointMatcher<float>::DataPoints conversion(cloudMatrix, labels);

	return conversion;
	}

}
}
}

/** @} */
