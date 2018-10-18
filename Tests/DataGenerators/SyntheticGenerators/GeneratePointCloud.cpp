/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file GeneratePointCloud.cpp
 * @date 16/10/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * This is the main program for generating point clouds from stereo camera pairs
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
#include "PointCloudGenerator.hpp"
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <fstream>

using namespace DataGenerators;
using namespace PoseWrapper;

const std::string USAGE =
" \n \
This program generates a set of point cloud by using the HirschmullerDisparityMapping DFN implementation from a set of stereo camera pair. It will also read ground truth poses from a file, and \
produce another output file that will contain the path to the point cloud and its relative pose. The output file can be used to run the point cloud assembly test. \n \n \
This program requires the following parameters: \n \
1. Path to the folder containing the image folder, the image list file an the pose file; \n \
2. the image list file: three comment lines and each following line contains an image pair in the form: time left_image_file_path right_image_file_path. \n \
3. the pose list file: three blank lines and each following line contains a pose in the form: time x y z qx qy qz qw \n \
4: the output folder path that will contain point clouds and output file \n \
5: the name of output file \n \
6: the configuration file path of Hirschmuller Disparity Mapping \n \
7: whether to enable plan filtering, this parameter is optional the default is no_plane_filtering, but it can also be set to plane_filtering, in which case the dominant plane is removed from the output point clouds. \n \n \
Example usage: ./generate_point_clouds ../tests/Data/ ImageList.txt PoseList.txt ../tests/Data/PointClouds/ output_cloud.txt ../../test/ConfigurationFiles/DFNs/StereoReconstruction/Hirsh_conf.yaml\n \n ";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 7, USAGE);
	std::string inputFolderPath = argv[1];
	std::string imageListFileName = argv[2];
	std::string poseListFileName = argv[3];
	std::string outputFolderPath = argv[4];
	std::string cloudListFileName = argv[5];
	std::string dfnConfigurationFile = argv[6];

	PointCloudGenerator generator(inputFolderPath, imageListFileName, poseListFileName, outputFolderPath, cloudListFileName, dfnConfigurationFile);
	if (argc >= 8)
		{
		std::string planeFilteringFlag = argv[7];
		if (planeFilteringFlag == "plane_filtering")
			{
			generator.EnablePlaneFiltering();
			}
		else
			{
			ASSERT(planeFilteringFlag == "no_plane_filtering", USAGE);
			}
		}


	generator.GenerateClouds();

	return 0;
	}

/** @} */
