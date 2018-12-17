/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file GuiTestReconstructionAndLocalisation.hpp
 * @date 31/08/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This is the main interface class for implementations of the DFPC Reconstruction 3D.
 * It performs some common operations
 *
 * @{
 */

#ifndef GUI_TEST_RECONSTRUCTION_AND_LOCALIZATION_HPP
#define GUI_TEST_RECONSTRUCTION_AND_LOCALIZATION_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <ReconstructionAndIdentification/ReconstructionAndIdentificationInterface.hpp>
#include <Errors/Assert.hpp>

#include <Converters/MatToFrameConverter.hpp>
#include <Types/CPP/Frame.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/PointCloud.hpp>

#include <Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Converters/SupportTypes.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class GuiTestReconstructionAndLocalisation
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		enum ImageFilesType
			{
			MONO_CAMERA,
			STEREO_CAMERA_ONE_FILE,
			STEREO_CAMERA_TWO_FILES
			};

		GuiTestReconstructionAndLocalisation(const std::string& configurationFilePath, const std::string& imageFilesFolder, const std::string& imagesListFileName, ImageFilesType imageFilesType,
			const std::string& modelFilePath);
		~GuiTestReconstructionAndLocalisation();

		void Run(CDFF::DFPC::ReconstructionAndIdentificationInterface& reconstructorAndIdentifier);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:


	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		std::string imageFilesFolder;
		std::string configurationFilePath;
		std::vector<std::string> imageFileNamesList;
		std::vector<std::string> secondImageFileNamesList;
		ImageFilesType imageFilesType;
		std::string modelFilePath;

		Converters::MatToFrameConverter frameConverter;
		Converters::PclPointCloudToPointCloudConverter pointCloudConverter;

		void LoadImagesList(const std::string& imagesListFileName);
		bool LoadNextImages(FrameWrapper::FrameConstPtr& leftImage, FrameWrapper::FrameConstPtr& rightImage);
		PointCloudWrapper::PointCloudConstPtr LoadPointCloud(const std::string& file);

	};

#endif

/* GuiTestReconstructionAndIdentification.hpp */
/** @} */
