/**
 * @author Irene Sanz
 */

/**
 * @addtogroup DFNs
 * @{
 */

#include "Octree.hpp"

#include <Converters/PclOctreeToOctreeConverter.hpp>
#include <Converters/FrameToMatConverter.hpp>

#include <Errors/Assert.hpp>
#include <fstream>
#include <stdlib.h>

namespace
{
    long mapValue(long x, long in_min, long in_max, long out_min, long out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}

namespace CDFF
{
namespace DFN
{
namespace Voxelization
{

//=====================================================================================================================
Octree::Octree()
{
	parametersHelper.AddParameter<double>("GeneralParameters", "MinDistance", parameters.minDistance, DEFAULT_PARAMETERS.minDistance);
    parametersHelper.AddParameter<double>("GeneralParameters", "MaxDistance", parameters.maxDistance, DEFAULT_PARAMETERS.maxDistance);
    parametersHelper.AddParameter<double>("GeneralParameters", "Resolution", parameters.resolution, DEFAULT_PARAMETERS.resolution);
	configurationFilePath = "";
}

//=====================================================================================================================
const Octree::OctreeOptionsSet Octree::DEFAULT_PARAMETERS =
{
	/*.minDistance =*/ 0.0,
	/*.maxDistance =*/ 100.0,
    /*.resolution =*/  1.0
};

//=====================================================================================================================
void Octree::configure()
{
	parametersHelper.ReadFile(configurationFilePath);
	ValidateParameters();
}

//=====================================================================================================================
void Octree::process()
{
    // Read data from input port
    cv::Mat input_image = Converters::FrameToMatConverter().Convert(&inDepth);

    // Process data
    ValidateInputs(input_image);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = convertToPointCloud(input_image);
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (parameters.resolution);
    octree.setInputCloud (input_cloud);
    octree.addPointsFromInputCloud();

    // Write data to output port
    outOctree = *Converters::PclOctreeToOctreeConverter().Convert(octree);
}

//=====================================================================================================================
pcl::PointCloud<pcl::PointXYZ>::Ptr Octree::convertToPointCloud(const cv::Mat& img)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    int i,j;
    for( i = 0; i < img.rows; ++i)
    {
        for ( j = 0; j < img.cols; ++j)
        {
            int depth =  static_cast<int>(img.at<uchar>(i,j));
            depth = ::mapValue(depth, 0, 255, parameters.minDistance, parameters.maxDistance);
            cloud->push_back(pcl::PointXYZ(i,j,depth));
        }
    }
    return cloud;
}

//=====================================================================================================================
void Octree::ValidateParameters()
{
    ASSERT(parameters.minDistance < parameters.maxDistance, "Octree Configuration Error: minDistance must be smaller than maxDistance");
    ASSERT(parameters.resolution > 0, "Octree Configuration Error: resolution must be larger than 0");
}

//=====================================================================================================================
void Octree::ValidateInputs(const cv::Mat& inputImage)
{
	ASSERT(inputImage.rows > 0 && inputImage.cols > 0, "Octree error: input image is empty");
}

}
}
}

/** @} */
