/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ScanlineOptimization.cpp
 * @date 24/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the DFN Scanline Optimization.
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
#include <StereoReconstruction/ScanlineOptimization.hpp>

using namespace dfn_ci;

const std::string USAGE =
	"This method takes up to eight optional parameters: \n \
	(i) The name of the configuration file of the DFN (the configuration file should be put in Tests/ConfigurationFiles/DFNs/StereoReconstruction/ folder) \n \
	(ii) The base folder containing the file list of images to process \n \
	(iii) the name of the file list of images (each line of the file should contain the path to two or three images from the base folder, two stareo images and \
	one optional disparity reference)) \n \
	(iv) whether you are using a disparity image as a reference, this option should be either UseReferenceDisparity or NoReferenceDisparity \n \
	(v) the path from the base folder representing the first part of the output disparity file name without the extension \n \
	(vi) the extension of the output disparity images \n \
	(vii) the path from the base folder representing the first part of the output point cloud file name without the extension \n \
	(viii) the extension of the output point cloud file, only '.ply' is currently supported \n \n \
	Example Usage: ./scanline_optimization_performance_test HirschmullerDisparityMapping_Performance_DeskReconstruction.yaml \
	/path/DataSetReconstruction/Desk ImagesListSingle.txt noReferenceDisparity Disparities/firstImage .jpg Clouds/outputCloud .ply \n \n \
	Note: if you do specify the output point cloud file, no point cloud will be saved. Likewise if you do no specify any image save file, no disparity image will be saved. \n";

int main(int argc, char** argv)
	{
	std::string configurationFileName = "ScanlineOptimization_Performance1.yaml";
	if (argc >= 2)
		{
		configurationFileName = argv[1];
		}

	StereoReconstructionInterface* reconstructor = new ScanlineOptimization();
	StereoReconstructionTestInterface interface("../tests/ConfigurationFiles/DFNs/StereoReconstruction", configurationFileName, "ScanlineOptimization.txt", reconstructor);
	
	if (argc >= 5)
		{
		std::string useReferenceDisparity = argv[4];
		interface.SetImageFilesPath(argv[2], argv[3], (useReferenceDisparity == "UseReferenceDisparity") );
		}

	if (argc >= 7)
		{
		interface.SetDisparityOutputFile(argv[5], argv[6]);
		}

	if (argc >= 9)
		{
		interface.SetCloudOutputFile(argv[7], argv[8]);
		}

	interface.Run();
	};

/** @} */
