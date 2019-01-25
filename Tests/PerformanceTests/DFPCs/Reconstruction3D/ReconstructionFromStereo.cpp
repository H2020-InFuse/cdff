/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ReconstructionFromStereo.cpp
 * @date 23/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the DFPC Reconstruction3D with implementation ReconstructionFromStereo.
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
#include <Reconstruction3D/ReconstructionFromStereo.hpp>
#include "Reconstruction3D.hpp"

using namespace CDFF::DFPC::Reconstruction3D;

const std::string USAGE =
	"This method takes five parameters: \n \
	(i) the folder path to the file containing the DFPC configuration file \n \
	(ii) the name of the DFPC configuration file \n \
	(iii) the name of the output performance file (that will be located in the DFPC configuration file folder) \n \
	(iv) The folder path to the file containing a list of image files \n \
	(v) the name of the file containing the list of image files: it should contain a row for each pair of images \n \n \
	There is one more optional parameter that you should set only if you want to save the output point cloud to file: \n \
	(iii) the file path to the point cloud without extension\n \n \
	Example Usage: ./reconstruction_from_stereo_performance_test ../../test/ConfigurationFiles/DFPCs/Reconstruction3D configuration1.yaml output.txt \
	/path/DataSetReconstruction/Desk ImagesListSingle.txt Clouds/outputCloud \n \n";


int main(int argc, char** argv)
	{
	std::string baseFolderPath, imagesListFileName;
	std::string outputCloudFileBaseName;//, outputCloudFileExtension;
	std::string configurationFolderPath, configurationFileName, outputFileName;

	ASSERT(argc >= 6, USAGE)
	configurationFolderPath = argv[1];
	configurationFileName = argv[2];
	outputFileName = argv[3];
	baseFolderPath = argv[4];
	imagesListFileName = argv[5];

	ReconstructionFromStereo* reconstructionFromStereo = new ReconstructionFromStereo;
	Reconstruction3DTestInterface interface
		(
		configurationFolderPath, 
		configurationFileName, 
		outputFileName,
		reconstructionFromStereo
		);
	interface.SetImageFilesPath(baseFolderPath, imagesListFileName);
	
	if (argc == 5)
		{
		outputCloudFileBaseName = argv[3];
		//outputCloudFileExtension = argv[4]; // not used
		interface.SetCloudOutputFile(outputCloudFileBaseName);
		}	

	interface.Run();
	};

/** @} */
