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

using namespace dfn_ci;

int main(int argc, char** argv)
	{
	std::string configurationFileName = "ImageUndistortion_Conf1.yaml";
	if (argc >= 2)
		{
		configurationFileName = argv[1];
		}

	ImageFilteringInterface* filter = new ImageUndistortion;
	ImageFilteringTestInterface interface("../tests/ConfigurationFiles/DFNs/ImageFiltering", configurationFileName, "ImageUndistortionOutput.txt", filter);
	
	if (argc >= 4)
		{
		interface.SetImageFilePath(argv[2], argv[3]);
		}

	if (argc >= 6)
		{
		interface.SetOutputFile(argv[4], argv[5]);
		}

	interface.Run();
	};

/** @} */
