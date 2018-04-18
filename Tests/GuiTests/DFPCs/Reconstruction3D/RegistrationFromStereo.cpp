/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file RegistrationFromStereo.cpp
 * @date 18/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFPCsTest
 * 
 * Testing application for the DFPC RegistrationFromStereo.
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
#include "GuiTestReconstruction3D.hpp"
#include <Visualizers/OpencvVisualizer.hpp>
#include <Visualizers/PclVisualizer.hpp>
#include <Reconstruction3D/RegistrationFromStereo.hpp>
#include <Errors/Assert.hpp>


using namespace dfpc_ci;

const std::string CONFIGURATION_FILE_PATH = "../../tests/ConfigurationFiles/DFPCs/Reconstruction3D/DfpcRegistrationFromStereo_conf01.yaml";
const std::string IMAGE_FILES_FOLDER = "../../tests/Data/Images/";
const std::string IMAGES_LIST_FILE_NAME = "imagesListStereo.txt";
const std::string IMAGE_FILES_TYPE = "stereocamera_oneimage";

GuiTestReconstruction3D::ImageFilesType StringToImageFilesType(std::string string)
	{
	if (string == "monocamera" || string == "mono" || string == "camera" || string == "singlecamera" || string == "Monocamera" || string == "Mono" || string == "Camera")
		{
		ASSERT(false, "This gui test does not support the use of monocamera. Choose one of: stereocamera_oneimage, or stereocamera_twoimages");
		return GuiTestReconstruction3D::MONO_CAMERA;
		}
	else if (string == "stereocamera_oneimage" || string == "stereo_one" || string == "stereocamera_one" || string == "stereo_oneimage")
		{
		return GuiTestReconstruction3D::STEREO_CAMERA_ONE_FILE;
		}
	else if (string == "stereocamera_twoimages" || string == "stereo_two" || string == "stereocamera_two" || string == "stereo_twoimages")
		{
		return GuiTestReconstruction3D::STEREO_CAMERA_TWO_FILES;
		}
	else
		{
		ASSERT(false, "4th parameters needs to be one between: stereocamera_oneimage, or stereocamera_twoimages");
		}
	}

int main(int argc, char** argv)
	{
	Visualizers::OpencvVisualizer::Enable();
	Visualizers::PclVisualizer::Enable();

	ASSERT(argc == 1 || argc == 5, "Please provide four paramters: ConfigurationFilePath, ImageFilesFolder, ImagesListFileName and ImageFilesType");

	std::string configurationFilePath = (argc == 5) ? argv[1] : CONFIGURATION_FILE_PATH;
	std::string imageFilesFolder = (argc == 5) ? argv[2] : IMAGE_FILES_FOLDER;
	std::string imagesListFileName = (argc == 5) ? argv[3] : IMAGES_LIST_FILE_NAME;
	std::string imageFilesType = (argc == 5) ? argv[4] : IMAGE_FILES_TYPE;

	GuiTestReconstruction3D guiReconstruction3d
		(
		configurationFilePath, 
		imageFilesFolder,
		imagesListFileName,
		StringToImageFilesType(imageFilesType)
		);

	RegistrationFromStereo registrationFromStereo;

	guiReconstruction3d.Run(registrationFromStereo);

	return 0;
	};

/** @} */
