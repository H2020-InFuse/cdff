/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file RegistrationAndMatching.cpp
 * @date 31/08/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFPCsTest
 * 
 * Testing application for the DFPC RegistrationAndMatching.
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
#include "GuiTestReconstructionAndIdentification.hpp"
#include <Visualizers/OpenCVVisualizer.hpp>
#include <Visualizers/PCLVisualizer.hpp>
#include <ReconstructionAndIdentification/RegistrationAndMatching.hpp>
#include <Errors/Assert.hpp>


using namespace CDFF::DFPC::ReconstructionAndIdentification;

const std::string CONFIGURATION_FILE_PATH = "../../tests/ConfigurationFiles/DFPCs/Reconstruction3D/DfpcAdjustmentFromStereo_conf01.yaml";
const std::string IMAGE_FILES_FOLDER = "../../tests/Data/Images/";
const std::string IMAGES_LIST_FILE_NAME = "imagesListStereo.txt";
const std::string IMAGE_FILES_TYPE = "stereocamera_oneimage";
const std::string MODEL_FILE_PATH = "path/to/model/file";

GuiTestReconstructionAndLocalisation::ImageFilesType StringToImageFilesType(std::string string)
	{
	if (string == "monocamera" || string == "mono" || string == "camera" || string == "singlecamera" || string == "Monocamera" || string == "Mono" || string == "Camera")
		{
		ASSERT(false, "This gui test does not support the use of monocamera. Choose one of: stereocamera_oneimage, or stereocamera_twoimages");
		return GuiTestReconstructionAndLocalisation::MONO_CAMERA;
		}
	else if (string == "stereocamera_oneimage" || string == "stereo_one" || string == "stereocamera_one" || string == "stereo_oneimage")
		{
		return GuiTestReconstructionAndLocalisation::STEREO_CAMERA_ONE_FILE;
		}
	else if (string == "stereocamera_twoimages" || string == "stereo_two" || string == "stereocamera_two" || string == "stereo_twoimages")
		{
		return GuiTestReconstructionAndLocalisation::STEREO_CAMERA_TWO_FILES;
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

	ASSERT(argc >= 6, "Please provide four paramters: ConfigurationFilePath, ImageFilesFolder, ImagesListFileName, ImageFilesType, and modelFilePath");

	std::string configurationFilePath = (argc >= 6) ? argv[1] : CONFIGURATION_FILE_PATH;
	std::string imageFilesFolder = (argc >= 6) ? argv[2] : IMAGE_FILES_FOLDER;
	std::string imagesListFileName = (argc >= 6) ? argv[3] : IMAGES_LIST_FILE_NAME;
	std::string imageFilesType = (argc >= 6) ? argv[4] : IMAGE_FILES_TYPE;
	std::string modelFilePath = (argc >= 6) ? argv[5] : MODEL_FILE_PATH;

	GuiTestReconstructionAndLocalisation guiReconstruction3d
		(
		configurationFilePath, 
		imageFilesFolder,
		imagesListFileName,
		StringToImageFilesType(imageFilesType),
		modelFilePath
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

	RegistrationAndMatching* registrationAndLocalisation = new RegistrationAndMatching;
	guiReconstruction3d.Run(*registrationAndLocalisation);

	delete(registrationAndLocalisation);
	return 0;
	};

/** @} */
