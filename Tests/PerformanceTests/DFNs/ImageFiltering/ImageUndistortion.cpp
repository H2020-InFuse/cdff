/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageUndistortion.cpp
 * @date 25/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Performance Test for the DFN Image Undistortion.
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
#include "ImageFiltering.hpp"
#include <ImageFiltering/ImageUndistortion.hpp>

using namespace CDFF::DFN;
using namespace CDFF::DFN::ImageFiltering;

const std::string USAGE =
	"The program takes up to seven optional parameters: \n \
	(i) the folder path to the DFN configuration file \n \
	(ii) The name of the configuration file of the DFN \n \
	(iii) the name of the output performance report file (that wil be located in the folder of the DFN configuration file) \n \
	(iv) the base folder containing the images List file \n \
	(v) the name of the images List file containing the relative path to one image in each line \n \
	(vi) the relative path from the base folder to the output image, the name of the image should not contain the image extension \n \
	(vii) the extension of theoutput images \n \n \
	Example Usage: ./image_undistortion_performance_test ../tests/ConfigurationFiles/DFNs/ImageFiltering ImageUndistortion_conf.yaml output.txt \
	/path/Set/Images ImagesList.txt ../RectifiedImages/image .jpg \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 6, USAGE)
	
	std::string configurationFileFolderPath, configurationFileName, outputFileName;
	std::string imageListFolderPath, imageListFileName;
	configurationFileFolderPath = argv[1];
	configurationFileName = argv[2];
	outputFileName = argv[3];
	imageListFolderPath = argv[4];
	imageListFileName = argv[5];

	ImageUndistortion* filter = new ImageUndistortion;
	ImageFilteringTestInterface interface(configurationFileFolderPath, configurationFileName, outputFileName, filter);
	interface.SetImageFilePath(imageListFolderPath,imageListFileName);

	if (argc >= 8)
		{
		interface.SetOutputFile(argv[6], argv[7]);
		}

	interface.Run();
	};

/** @} */
