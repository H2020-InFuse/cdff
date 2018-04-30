/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageUndistortionRectification.cpp
 * @date 23/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the DFN Image Undistortion Rectification.
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
#include <ImageFiltering/ImageUndistortionRectification.hpp>


using namespace dfn_ci;

int main(int argc, char** argv)
	{
	std::string configurationFileName = "ImageUndistortionRectification_Conf1.yaml";
	if (argc >= 2)
		{
		configurationFileName = argv[1];
		}

	ImageFilteringInterface* filter = new ImageUndistortionRectification;
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
