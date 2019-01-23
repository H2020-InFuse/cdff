/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HirschmullerDisparityMapping.cpp
 * @date 19/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Performance Test for the DFN Hirschmuller Disparity Mapping.
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
#include "StereoReconstruction.hpp"
#include <StereoReconstruction/HirschmullerDisparityMapping.hpp>

using namespace CDFF::DFN;
using namespace CDFF::DFN::StereoReconstruction;

const std::string USAGE =
	"This method takes up to ten optional parameters: \n \
	(i) the folder path to the DFN configuration file \n \
	(ii) The name of the configuration file of the DFN \n \
	(iii) the name of the output performance report file (that wil be located in the folder of the DFN configuration file) \n \
	(iv) The base folder containing the file list of images to process \n \
	(v) the name of the file list of images (each line of the file should contain the path to two or three images from the base folder, two stareo images and \
	one optional disparity reference)) \n \
	(vi) whether you are using a disparity image as a reference, this option should be either UseReferenceDisparity or NoReferenceDisparity \n \
	(vii) the path from the base folder representing the first part of the output disparity file name without the extension \n \
	(viii) the extension of the output disparity images \n \
	(ix) the path from the base folder representing the first part of the output point cloud file name without the extension \n \
	(x) the extension of the output point cloud file, only '.ply' is currently supported \n \n \
	Example Usage: ./hirschmuller_disparity_mapping_performance_test ../../test/ConfigurationFiles/DFNs/StereoReconstruction HirschmullerDisparityMapping_Performance.yaml output.txt\
	/path/DataSetReconstruction/Desk ImagesListSingle.txt noReferenceDisparity Disparities/firstImage .jpg Clouds/outputCloud .ply \n \n \
	Note: if you do specify the output point cloud file, no point cloud will be saved. Likewise if you do no specify any image save file, no disparity image will be saved. \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 7, USAGE)
	
	std::string configurationFileFolderPath, configurationFileName, outputFileName;
	std::string imageListFolderPath, imageListFileName, useReferenceDisparity;
	configurationFileFolderPath = argv[1];
	configurationFileName = argv[2];
	outputFileName = argv[3];
	imageListFolderPath = argv[4];
	imageListFileName = argv[5];
	useReferenceDisparity = argv[6];

	StereoReconstructionInterface* reconstructor = new HirschmullerDisparityMapping();
	StereoReconstructionTestInterface interface(configurationFileFolderPath, configurationFileName, outputFileName, reconstructor);

	ASSERT(useReferenceDisparity == "UseReferenceDisparity" || useReferenceDisparity == "NoReferenceDisparity",
		"Error: 6th parameter has to be either UseReferenceDisparity or NoReferenceDisparity");
	interface.SetImageFilesPath(imageListFolderPath, imageListFileName, (useReferenceDisparity == "UseReferenceDisparity") );

	if (argc >= 9)
		{
		interface.SetDisparityOutputFile(argv[7], argv[8]);
		}

	if (argc >= 11)
		{
		interface.SetCloudOutputFile(argv[9], argv[10]);
		}

	interface.Run();
	};

/** @} */
