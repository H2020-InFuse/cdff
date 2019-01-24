/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisShotIcp.cpp
 * @date 20/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the integration of Harris 3d detector, Shot 3d descriptor, and Icp matcher 3d.
 * This program will execute the three DFNs multiple times on the same set of input point clouds according to different parameters.
 * The parameters are specified in the yaml configuration file for the three DFNs.
 * The input points cloud and ground truth are specified in the program call command line.
 * The output will be available in file format and will contain the error between the estimated position of the models and the ground truth.
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "DetectionDescriptionMatching3D.hpp"

#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <FeaturesMatching3D/Icp3D.hpp>

using namespace CDFF::DFN::FeaturesExtraction3D;
using namespace CDFF::DFN::FeaturesDescription3D;
using namespace CDFF::DFN::FeaturesMatching3D;

const std::string USAGE =
	" \n \
	This programs requires four parameters: \n \
	(i) the folder path containing the configuration files of each DFN \n \
	(ii) the name of the Harris3D Configuration file \n \
	(iii) the name of the Shot3D Configuration file \n \
	(iv) the name of the Icp3D Configuration file \n \
	(v) the name of the output performance file (that will be containing in the configuration folder) \n \
	(vi) the scene point cloud path in ply format; \n \
	(vii) the size of the voxel grid used as a preliminary filter. If the value is 0 the filter is disabled.; \n \
	(viii) the file path containing the transform for each model. There is one line for each transform in format 'x y z qx qy qz qw'. \n \
	(ix + ) the following parameters (9th, 10th etc..) are the file paths to the model clouds. \n \n \
	Example Usage: ./harris_shot_icp ../tests/ConfigurationFiles/DFNsIntegration/Odometry3D HarrisDetector3d_PerformanceTest_1.yaml ShotDescriptor3d_PerformanceTest_1.yaml \
	Icp3d_PerformanceTest_1.yaml Harris_Shot_Icp.txt ../tests/Data/PointClouds/bunny0.ply 0.001 ../tests/Data/PointCloud/Transform.txt ../test/Data/PointCloud/bunnyPart1.ply  \
	../test/Data/PointCloud/bunnyPart2.ply \n \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 10, USAGE)
	std::string configurationFolderPath, harrisConfigurationFileName, shotConfigurationFileName, ransacConfigurationFileName, outputFileName;
	std::string sceneCloudFilePath, transformFilePath;
	float voxelGridFilterSize;
	configurationFolderPath = argv[1];
	harrisConfigurationFileName = argv[2];
	shotConfigurationFileName = argv[3];
	ransacConfigurationFileName = argv[4];
	outputFileName = argv[5];
	sceneCloudFilePath = argv[6];
	transformFilePath = argv[8];
	
	try 
		{
		voxelGridFilterSize = std::stof(argv[7]);
		}
	catch (...)
		{
		ASSERT(false, "7th parameter has to be a float");
		}

	
	std::vector<std::string> baseConfigurationFiles =
		{
		harrisConfigurationFileName,
		shotConfigurationFileName,
		ransacConfigurationFileName
		};

	DetectionDescriptionMatching3DTestInterface::DFNsSet dfnsSet;
	dfnsSet.extractor = new HarrisDetector3D();
	dfnsSet.descriptor = new ShotDescriptor3D();
	dfnsSet.matcher = new Icp3D();
	DetectionDescriptionMatching3DTestInterface interface(configurationFolderPath, baseConfigurationFiles, outputFileName, dfnsSet);

	interface.SetInputCloud(sceneCloudFilePath, voxelGridFilterSize);

	std::vector<std::string> modelsFilesList;
	for(int argvIndex = 9; argvIndex < argc; argvIndex++)
		{
		modelsFilesList.push_back(argv[argvIndex]);
		}
	interface.SetModelsCloud(transformFilePath, modelsFilesList);

	interface.Run();
	};

/** @} */
