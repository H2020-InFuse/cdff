/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageUndistortionRectification.cpp
 * @date 09/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Testing application for the DFN ImageUndistortionRectification.
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
#include <ImageFiltering/ImageUndistortionRectification.hpp>

#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Errors/Assert.hpp>

#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

using namespace CDFF::DFN::ImageFiltering;
using namespace Converters;
using namespace FrameWrapper;

class ImageUndistortionRectificationTestInterface : public DFNTestInterface
{
	public:
		ImageUndistortionRectificationTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~ImageUndistortionRectificationTestInterface();

	private:
		enum CameraPosition
		{
			LEFT_CAMERA,
			RIGHT_CAMERA,
		};

		ImageUndistortionRectification* undistort;

		cv::Mat cvImage;
		FrameConstPtr inputImage;
		std::string outputWindowName;
		CameraPosition cameraPosition;
		bool saveResultToFile;

		void SetupParameters();
		void DisplayResult();
};

ImageUndistortionRectificationTestInterface::ImageUndistortionRectificationTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight), inputImage()
{
	cameraPosition = LEFT_CAMERA;
	saveResultToFile = false;

	undistort = new ImageUndistortionRectification();
	SetDFN(undistort);

	cv::Mat doubleImage = cv::imread("../../tests/Data/Images/chair40.png", cv::IMREAD_COLOR);
	if (cameraPosition == LEFT_CAMERA)
	{
		cvImage = doubleImage(cv::Rect(0,0,doubleImage.cols/2, doubleImage.rows));
	}
	else
	{
		cvImage = doubleImage(cv::Rect(doubleImage.cols/2,0,doubleImage.cols/2, doubleImage.rows));
	}
	inputImage = MatToFrameConverter().Convert(cvImage);

	undistort->imageInput(*inputImage);
	outputWindowName = "Image Undistortion Rectification Result";
}

ImageUndistortionRectificationTestInterface::~ImageUndistortionRectificationTestInterface()
{
	delete(undistort);
	delete(inputImage);
}

void ImageUndistortionRectificationTestInterface::SetupParameters()
{
	AddParameter("GeneralParameters", "ConstantBorderValue", 0, 100, 1e-2);
	AddParameter("GeneralParameters", "InterpolationMethod", 0, 3);
	AddParameter("GeneralParameters", "BorderMode", 0, 2);
	AddParameter("GeneralParameters", "CameraConfigurationMode", 0, 2);

	AddParameter("ImageSize", "Width", 1280, 1280);
	AddParameter("ImageSize", "Height", 720, 720);

	if (cameraPosition == LEFT_CAMERA)
	{
		AddParameter("CameraMatrix", "FocalLengthX", 693.4181807813, 700, 1e-5);
		AddParameter("CameraMatrix", "FocalLengthY", 690.36629049483, 700, 1e-5);
		AddParameter("CameraMatrix", "PrinciplePointX", 671.7716154809, 700, 1e-5);
		AddParameter("CameraMatrix", "PrinciplePointY", 391.33378485796, 700, 1e-5);

		AddParameter("Distortion", "UseK3", 1, 1);
		AddParameter("Distortion", "UseK4ToK6", 1, 1);
		AddParameter("Distortion", "K1", 0.39160333319788, 100, 1e-5);
		AddSignedParameter("Distortion", "K2", -67.273810332838, 100, 1e-5);
		AddParameter("Distortion", "P1", 0.00056073969847596, 100, 1e-5);
		AddSignedParameter("Distortion", "P2", -0.0035917800798291, 100, 1e-5);
		AddParameter("Distortion", "K3", 237.61237318275, 300, 1e-5);
		AddParameter("Distortion", "K4", 0.35372515932617, 100, 1e-5);
		AddSignedParameter("Distortion", "K5", -66.934609418439, 100, 1e-5);
		AddParameter("Distortion", "K6", 236.75743075463, 300, 1e-5);

		AddParameter("RectificationMatrix", "Element_0_0", 0.99998692232279, 1, 1e-8);
		AddParameter("RectificationMatrix", "Element_0_1", 0.00045433768478094, 1, 1e-8);
		AddSignedParameter("RectificationMatrix", "Element_0_2", -0.0050939926049538, 1, 1e-8);
		AddSignedParameter("RectificationMatrix", "Element_1_0", -0.00046438147916384, 1, 1e-8);
		AddParameter("RectificationMatrix", "Element_1_1", 0.999997950374, 1, 1e-8);
		AddSignedParameter("RectificationMatrix", "Element_1_2", -0.0019706845597646, 1, 1e-8);
		AddParameter("RectificationMatrix", "Element_2_0", 0.0050930868079138, 1, 1e-8);
		AddParameter("RectificationMatrix", "Element_2_1", 0.0019730243436088, 1, 1e-8);
		AddParameter("RectificationMatrix", "Element_2_2", 0.99998508370961, 1, 1e-8);
	}
	else
	{
		AddParameter("CameraMatrix", "FocalLengthX", 693.702285943993, 700, 1e-5);
		AddParameter("CameraMatrix", "FocalLengthY", 691.5964653592, 700, 1e-5);
		AddParameter("CameraMatrix", "PrinciplePointX", 672.36931687204, 700, 1e-5);
		AddParameter("CameraMatrix", "PrinciplePointY", 393.60885185491, 700, 1e-5);

		AddParameter("Distortion", "UseK3", 1, 1);
		AddParameter("Distortion", "UseK4ToK6", 1, 1);
		AddSignedParameter("Distortion", "K1", -11.395471000651, 100, 1e-5);
		AddParameter("Distortion", "K2", 157.12777744394, 200, 1e-5);
		AddParameter("Distortion", "P1", 0.00038932692838161, 100, 1e-5);
		AddSignedParameter("Distortion", "P2", -0.0028309139682407, 100, 1e-5);
		AddSignedParameter("Distortion", "K3", -79.419861245485, 300, 1e-5);
		AddSignedParameter("Distortion", "K4", -11.374087198325, 100, 1e-5);
		AddParameter("Distortion", "K5", 157.10782368884, 200, 1e-5);
		AddSignedParameter("Distortion", "K6", -79.515010847945, 200, 1e-5);

		AddParameter("RectificationMatrix", "Element_0_0", 0.99999671131107, 1, 1e-8);
		AddParameter("RectificationMatrix", "Element_0_1", 0.0001020440908808, 1, 1e-8);
		AddSignedParameter("RectificationMatrix", "Element_0_2", -0.0025626068846276, 1, 1e-8);
		AddSignedParameter("RectificationMatrix", "Element_1_0", -9.6990784025898e-05, 1, 1e-8);
		AddParameter("RectificationMatrix", "Element_1_1", 0.99999805092854, 1, 1e-8);
		AddSignedParameter("RectificationMatrix", "Element_1_2", 0.0019719867945831, 1, 1e-8);
		AddParameter("RectificationMatrix", "Element_2_0", 0.0025628031195234, 1, 1e-8);
		AddSignedParameter("RectificationMatrix", "Element_2_1", -0.0019717317600811, 1, 1e-8);
		AddParameter("RectificationMatrix", "Element_2_2", 0.99999477214335, 1, 1e-8);
	}
}

void ImageUndistortionRectificationTestInterface::DisplayResult()
{
	const Frame& undistortedImage = undistort->imageOutput();
	cv::Mat undistortedCvImage = FrameToMatConverter().Convert(&undistortedImage);

	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::imshow(outputWindowName, undistortedCvImage);
	cv::namedWindow("original", CV_WINDOW_NORMAL);
	cv::imshow("original", cvImage);
	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );

	if (saveResultToFile)
	{
		if (cameraPosition == LEFT_CAMERA)
		{
			cv::imwrite("../../tests/Data/Images/RectifiedChair40Left.png", undistortedCvImage);
		}
		else
		{
			cv::imwrite("../../tests/Data/Images/RectifiedChair40Right.png", undistortedCvImage);
		}
	}
}

int main(int argc, char** argv)
{
	ImageUndistortionRectificationTestInterface interface("ImageUndistortionRectification", 100, 40);
	interface.Run();
};

/** @} */
