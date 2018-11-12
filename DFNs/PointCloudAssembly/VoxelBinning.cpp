/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "VoxelBinning.hpp"

#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <stdlib.h>
#include <fstream>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

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

VoxelBinning::VoxelBinning()
{
	parametersHelper.AddParameter<float>("GeneralParameters", "VoxelResolution", parameters.voxelResolution, DEFAULT_PARAMETERS.voxelResolution);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseIncrementalMode", parameters.useIncrementalMode, DEFAULT_PARAMETERS.useIncrementalMode);
	parametersHelper.AddParameter<bool>("GeneralParameters", "UseDistanceFilter", parameters.useDistanceFilter, DEFAULT_PARAMETERS.useDistanceFilter);

	configurationFilePath = "";
	storedCloud = NULL;
	assembledCloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZ> >(*( new pcl::PointCloud<pcl::PointXYZ> ));

	SetPosition(inViewCenter, 0, 0, 0);
	inViewRadius = 100;
}

VoxelBinning::~VoxelBinning()
{
}

void VoxelBinning::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();

	if (parameters.useIncrementalMode && storedCloud == NULL)
		{
		storedCloud = boost::make_shared< pcl::PointCloud<pcl::PointXYZ> >(*( new pcl::PointCloud<pcl::PointXYZ> ));
		}
}

void VoxelBinning::process()
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

	AssemblePointCloud(); 
	PrepareOutAssembledPointCloud();

	if (parameters.useIncrementalMode)
		{
		storedCloud->points.resize(0);
		*storedCloud += *assembledCloud;
		}
}

const VoxelBinning::VoxelBinningOptionsSet VoxelBinning::DEFAULT_PARAMETERS
{
	/*.voxelResolution =*/ 0.01,
	/*.useIncrementalMode =*/ false,
	/*.useDistanceFilter =*/ false
};

void VoxelBinning::AssemblePointCloud()
	{
	assembledCloud->points.resize(0);
	*assembledCloud += *firstCloud;
	*assembledCloud += *secondCloud;
	pcl::VoxelGrid< pcl::PointXYZ > voxelGrid;
	voxelGrid.setInputCloud (assembledCloud);
	voxelGrid.setLeafSize (parameters.voxelResolution, parameters.voxelResolution, parameters.voxelResolution);
	voxelGrid.filter (*assembledCloud);
	}

void VoxelBinning::PrepareOutAssembledPointCloud()
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

void VoxelBinning::ValidateParameters()
{
	ASSERT( parameters.voxelResolution > 0, "VoxelBinning Configuration error, maxNeighbourDistance is not positive");
}

}
}
}

/** @} */
