/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "StatisticalOutlierRemoval.hpp"

#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <stdlib.h>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace Converters;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;
using namespace PoseWrapper;
using namespace BaseTypesWrapper;

namespace CDFF
{
namespace DFN
{
namespace PointCloudFiltering
{

StatisticalOutlierRemoval::StatisticalOutlierRemoval()
{
	parametersHelper.AddParameter<int>("GeneralParameters", "NumberOfNearestNeighbours", parameters.numberOfNearestNeighbours, DEFAULT_PARAMETERS.numberOfNearestNeighbours);
	parametersHelper.AddParameter<double>("GeneralParameters", "StandardDeviationMultiplier", parameters.standardDeviationMultiplier, DEFAULT_PARAMETERS.standardDeviationMultiplier);
	parametersHelper.AddParameter<bool>("GeneralParameters", "TakeOutliersOnly", parameters.takeOutliersOnly, DEFAULT_PARAMETERS.takeOutliersOnly);

	configurationFilePath = "";
}

StatisticalOutlierRemoval::~StatisticalOutlierRemoval()
{
}

void StatisticalOutlierRemoval::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

void StatisticalOutlierRemoval::process()
{
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud = pointCloudToPclPointCloud.Convert(&inPointCloud);

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr filteredCloud = FilterCloud(cloud);

	PointCloudConstPtr filteredCloudOutput = pclPointCloudToPointCloud.Convert(filteredCloud);
	Copy(*filteredCloudOutput, outFilteredPointCloud);
	delete(filteredCloudOutput);
}

const StatisticalOutlierRemoval::StatisticalOutlierRemovalOptionsSet StatisticalOutlierRemoval::DEFAULT_PARAMETERS
{
	/*.numberOfNearestNeighbours =*/ 50,
	/*.standardDeviationMultiplier =*/ 1.0,
	/*.takeOutliersOnly =*/ false
};

pcl::PointCloud<pcl::PointXYZ>::ConstPtr StatisticalOutlierRemoval::FilterCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
	filter.setInputCloud (cloud);
	filter.setMeanK (parameters.numberOfNearestNeighbours);
	filter.setStddevMulThresh (parameters.standardDeviationMultiplier);
	filter.setNegative(parameters.takeOutliersOnly);
	filter.filter (*filteredCloud);

	return filteredCloud;
	}

void StatisticalOutlierRemoval::ValidateParameters()
	{
	ASSERT(parameters.standardDeviationMultiplier >= 0, "StatisticalOutlierRemoval parameters error, standardDeviationMultiplier has to be non-negative");
	ASSERT(parameters.numberOfNearestNeighbours > 0, "StatisticalOutlierRemoval parameters error, numberOfNearestNeighbours has to be positive");
	}

}
}
}

/** @} */
