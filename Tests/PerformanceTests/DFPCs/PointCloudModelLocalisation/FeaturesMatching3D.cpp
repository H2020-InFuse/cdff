/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesMatching3D.cpp
 * @date 22/01/2019
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the DFPC PointCloudModelLocalisation with implementation FeaturesMatching3D.
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
#include <PointCloudModelLocalisation/FeaturesMatching3D.hpp>
#include "PointCloudModelLocalisation.hpp"

using namespace CDFF::DFPC::PointCloudModelLocalisation;

const std::string USAGE =
	"This method takes two parameters: \n \
	(i) The folder path to the file containing a list of point clouds and poses \n \
	(ii) the name of the file containing the list of point clouds files: each row contains the scene cloud path, the model cloud path, and the pose file path \n \
	(iii) the folder path to the file containing the DFPC configuration file \n \
	(iv) the name of the DFPC configuration file \n \
	(v) the name of the output file that will be placed in the folder containing the DFPC configuration file \n \n \
	Example Usage: ./features_matching_3d_performance_test \
	/path/DataSetReconstruction/Desk CloudsAndPoses.txt \n \n";


int main(int argc, char** argv)
	{
	std::string baseFolderPath, cloudsListFileName;
	std::string configurationFolderPath, configurationFileName, outputFileName;

	ASSERT(argc >= 6, USAGE)
	baseFolderPath = argv[1];
	cloudsListFileName = argv[2];
	configurationFolderPath = argv[3];
	configurationFileName = argv[4];
	outputFileName = argv[5];

	FeaturesMatching3D* featuresMatching3D = new FeaturesMatching3D;
	PointCloudModelLocalisationTestInterface interface
		(
		configurationFolderPath, 
		configurationFileName, 
		outputFileName,
		featuresMatching3D
		);
	interface.SetInputFilesPath(baseFolderPath, cloudsListFileName);	

	interface.Run();
	};

/** @} */
