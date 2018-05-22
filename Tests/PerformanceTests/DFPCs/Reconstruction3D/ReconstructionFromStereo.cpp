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

using namespace dfpc_ci;

const std::string USAGE =
	"This method takes two parameters: \n \
	(i) The folder path to the file containing a list of image files \n \
	(ii) the name of the file containing the list of image files: it should contain a row for each pair of images \n \n \
	There is one more optional parameter that you should set only if you want to save the output point cloud to file: \n \
	(iii) the file path to the point cloud without extension\n \n \
	Example Usage: ./reconstruction_from_stereo_performance_test \
	/path/DataSetReconstruction/Desk ImagesListSingle.txt Clouds/outputCloud \n \n";


int main(int argc, char** argv)
	{
	std::string baseFolderPath, imagesListFileName;
	std::string outputCloudFileBaseName, outputCloudFileExtension;

	ASSERT(argc >= 3, USAGE)
	baseFolderPath = argv[1];
	imagesListFileName = argv[2];

	ReconstructionFromStereo* reconstructionFromStereo = new ReconstructionFromStereo;
	Reconstruction3DTestInterface interface
		(
		"../tests/ConfigurationFiles/DFPCs/Reconstruction3D", 
		"ReconstructionFromStereo_Performance1.yaml", 
		"ReconstructonFromStereoOutput.txt",
		reconstructionFromStereo
		);
	interface.SetImageFilesPath(baseFolderPath, imagesListFileName);
	
	if (argc == 5)
		{
		outputCloudFileBaseName = argv[3];
		outputCloudFileExtension = argv[4];
		interface.SetCloudOutputFile(outputCloudFileBaseName);
		}	

	interface.Run();
	};

/** @} */
