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
#include "DetectionDescriptionMatching3D.hpp"

#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <FeaturesMatching3D/Ransac3D.hpp>


using namespace dfn_ci;

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

	if (argc >= 3)
		{
		std::string inputCloudFile = argv[1];
		float voxelGridFilterSize = std::stof(argv[2]);
		interface.SetInputCloud(inputCloudFile, voxelGridFilterSize);
		}

	if (argc >= 5)
		{
		std::string transformsFile = argv[3];
		std::vector<std::string> modelsFilesList;
		for(unsigned argvIndex = 4; argvIndex < argc; argvIndex++)
			{
			modelsFilesList.push_back(argv[argvIndex]);
			}
		interface.SetModelsCloud(transformsFile, modelsFilesList);
		}

	interface.Run();
	};

/** @} */
