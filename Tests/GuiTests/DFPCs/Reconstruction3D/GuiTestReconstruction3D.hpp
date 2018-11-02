/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file GuiTestReconstruction3D.hpp
 * @date 28/03/2018
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

#ifndef GUI_TEST_RECONSTRUCTION_3D_HPP
#define GUI_TEST_RECONSTRUCTION_3D_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Reconstruction3D/Reconstruction3DInterface.hpp>
#include <Errors/Assert.hpp>

#include <Converters/MatToFrameConverter.hpp>
#include <Frame.hpp>
#include <Pose.hpp>
#include <PointCloud.hpp>

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
class GuiTestReconstruction3D
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

		GuiTestReconstruction3D(std::string configurationFilePath, std::string imageFilesFolder, std::string imagesListFileName, ImageFilesType imageFilesType);
		~GuiTestReconstruction3D();

		void Run(CDFF::DFPC::Reconstruction3DInterface& reconstructor3d);

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

		Converters::MatToFrameConverter frameConverter;

		void LoadImagesList(std::string imagesListFileName);
		bool LoadNextImages(FrameWrapper::FrameConstPtr& leftImage, FrameWrapper::FrameConstPtr& rightImage);

	};

#endif

/* GuiTestReconstruction3D.hpp */
/** @} */
