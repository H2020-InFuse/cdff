/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageUndistortionTestInterface.cpp
 * @date 19/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Testing application for the DFN ImageUndistortion.
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
#include <ImageFiltering/ImageUndistortion.hpp>

#include <Converters/FrameToMatConverter.hpp>
#include <Converters/MatToFrameConverter.hpp>
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

class ImageUndistortionTestInterface : public DFNTestInterface
{
	public:
		ImageUndistortionTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight);
		~ImageUndistortionTestInterface();

	private:
		ImageUndistortion undistort;

		cv::Mat cvImage;
		FrameConstPtr inputImage;
		std::string outputWindowName;

		void SetupParameters() override;
		void DisplayResult() override;
};

ImageUndistortionTestInterface::ImageUndistortionTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight) :
	DFNTestInterface(dfnName, buttonWidth, buttonHeight), 
	inputImage(),
	undistort()
{
	SetDFN(&undistort);

	cv::Mat doubleImage = cv::imread("../../tests/Data/Images/SmestechLab.jpg", cv::IMREAD_COLOR);
	cvImage = doubleImage(cv::Rect(doubleImage.cols/2,0,doubleImage.cols/2, doubleImage.rows));
	inputImage = MatToFrameConverter().Convert(cvImage);

	undistort.imageInput(*inputImage);
	outputWindowName = "Image Undistortion Result";
}

ImageUndistortionTestInterface::~ImageUndistortionTestInterface()
{
	delete(inputImage);
}

void ImageUndistortionTestInterface::SetupParameters()
{
	AddParameter("CameraMatrix", "FocalLengthX", 1415.631284126374, 1500, 1e-5);
	AddParameter("CameraMatrix", "FocalLengthY", 1408.026118461406, 1500, 1e-5);
	AddParameter("CameraMatrix", "PrinciplePointX", 1013.347852589407, 1500, 1e-5);
	AddParameter("CameraMatrix", "PrinciplePointY", 592.5031927882591, 1500, 1e-5);

	AddSignedParameter("Distortion", "K1", -5.700997352957169, 10, 1e-8);
	AddSignedParameter("Distortion", "K2", 9.016454056014156, 10, 1e-8);
	AddSignedParameter("Distortion", "K3", -2.465929585147688, 10, 1e-8);
	AddSignedParameter("Distortion", "K4", -5.560701053165053, 10, 1e-8);
	AddSignedParameter("Distortion", "K5", 8.264481221246962, 10, 1e-8);
	AddSignedParameter("Distortion", "K6", -1.458304668407831, 10, 1e-8);
	AddSignedParameter("Distortion", "P1", 0.001056515495810514, 10, 1e-8);
	AddSignedParameter("Distortion", "P2", 0.002542555946247054, 10, 1e-8);

	AddParameter("Distortion", "UseK3", 1, 1);
	AddParameter("Distortion", "UseK4ToK6", 1, 1);
}

void ImageUndistortionTestInterface::DisplayResult()
{
	const Frame& undistortedImage = undistort.imageOutput();
	cv::Mat undistortedCvImage = FrameToMatConverter().Convert(&undistortedImage);

	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::imshow(outputWindowName, undistortedCvImage);
	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
}

int main(int argc, char** argv)
{
	ImageUndistortionTestInterface* interface = new ImageUndistortionTestInterface("ImageUndistortion", 100, 40);
	interface->Run();
	delete(interface);
};

/** @} */
