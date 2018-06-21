/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SparseRegistrationFromStereo.cpp
 * @date 05/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFPCsTest
 * 
 * Testing application for the DFPC SparseRegistrationFromStereo.
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
#include <Reconstruction3D/SparseRegistrationFromStereo.hpp>
#include <Errors/Assert.hpp>


using namespace dfpc_ci;

const std::string CONFIGURATION_FILE_PATH = "../../tests/ConfigurationFiles/DFPCs/Reconstruction3D/DfpcSparseRegistrationFromStereo_conf01.yaml";
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

	ASSERT(argc == 1 || argc >= 5, "Please provide four paramters: ConfigurationFilePath, ImageFilesFolder, ImagesListFileName and ImageFilesType");

	std::string configurationFilePath = (argc >= 5) ? argv[1] : CONFIGURATION_FILE_PATH;
	std::string imageFilesFolder = (argc >= 5) ? argv[2] : IMAGE_FILES_FOLDER;
	std::string imagesListFileName = (argc >= 5) ? argv[3] : IMAGES_LIST_FILE_NAME;
	std::string imageFilesType = (argc >= 5) ? argv[4] : IMAGE_FILES_TYPE;

	GuiTestReconstruction3D guiReconstruction3d
		(
		configurationFilePath, 
		imageFilesFolder,
		imagesListFileName,
		StringToImageFilesType(imageFilesType)
		);

	if (argc >= 6)
		{
		std::string enableVisualization = argv[5];
		if (enableVisualization == "FALSE" || enableVisualization == "False" || enableVisualization == "false")
			{
			Visualizers::OpencvVisualizer::Disable();
			Visualizers::PclVisualizer::Disable();	
			}
		}
	
	if (argc >= 7)
		{
		std::string enableSaving = argv[6];
		if (enableSaving == "True" || enableSaving == "True" || enableSaving == "true")
			{
			Visualizers::PclVisualizer::EnableSaving();
			}		
		}

	SparseRegistrationFromStereo* registrationFromStereo = new SparseRegistrationFromStereo;
	guiReconstruction3d.Run(*registrationFromStereo);

	delete(registrationFromStereo);
	return 0;
	};

/** @} */
