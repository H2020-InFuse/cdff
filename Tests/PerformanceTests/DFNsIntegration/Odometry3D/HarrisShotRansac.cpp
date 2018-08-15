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
 * Performance Test for the integration of Harris 3d detector, Shot 3d descriptor, and Ransac matcher 3d.
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
#include <FeaturesMatching3D/Ransac3D.hpp>

const std::string USAGE =
" \n \
This programs requires four parameters: \n \
(i) the 1st parameter is the scene point cloud path in ply format; \n \
(ii) the 2nd parameter is the size of the voxel grid used as a preliminary filter. If the value is 0 the filter is disabled.; \n \
(iii) the file path containing the transform for each model. There is one line for each transform in format 'x y z qx qy qz qw'. \n \
(iv + ) the following parameters (4th, 5th etc..) are the file paths to the model cloud. \n \n \
In addition to the four parameters you will need to configure the yaml configuration file for each DFN, they are: HarrisDetector3d_PerformanceTest_1.yaml, \
ShotDescriptor3d_PerformanceTest_1.yaml, Ransac3d_PerformanceTest_1.yaml. They should be located in Tests/ConfigurationFiles/DFNsIntegration/Odometry3D/ folder \n \n \
The output of the execution will be located in build/Tests/tests/ConfigurationFiles/DFNsIntegration/Odometry3D/Harris_Shot_Icp.txt \n \n \
Example Usage: ./harris_shot_ransac ../tests/Data/PointClouds/bunny0.ply 0.001 ../tests/Data/PointCloud/Transform.txt ../test/Data/PointCloud/bunnyPart1.ply  ../test/Data/PointCloud/bunnyPart2.ply \n \n";

using namespace CDFF::DFN::WHICH-DFN(S)-IF-ANY?;

int main(int argc, char** argv)
	{
	std::vector<std::string> baseConfigurationFiles =
		{
		"HarrisDetector3d_PerformanceTest_1.yaml",
		"ShotDescriptor3d_PerformanceTest_1.yaml",
		"Ransac3d_PerformanceTest_1.yaml"
		};

	DetectionDescriptionMatching3DTestInterface::DFNsSet dfnsSet;
	dfnsSet.extractor = new HarrisDetector3D();
	dfnsSet.descriptor = new ShotDescriptor3D();
	dfnsSet.matcher = new Ransac3D();
	DetectionDescriptionMatching3DTestInterface interface("../tests/ConfigurationFiles/DFNsIntegration/Odometry3D", baseConfigurationFiles, "Harris_Shot_Ransac.txt", dfnsSet);

	ASSERT(argc >= 5, USAGE);

	std::string inputCloudFile = argv[1];
	float voxelGridFilterSize = std::stof(argv[2]);
	interface.SetInputCloud(inputCloudFile, voxelGridFilterSize);

	std::string transformsFile = argv[3];
	std::vector<std::string> modelsFilesList;
	for(unsigned argvIndex = 4; argvIndex < argc; argvIndex++)
		{
		modelsFilesList.push_back(argv[argvIndex]);
		}
	interface.SetModelsCloud(transformsFile, modelsFilesList);

	interface.Run();
	};

/** @} */
