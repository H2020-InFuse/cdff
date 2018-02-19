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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <ImageFiltering/ImageUndistortion.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToFrameConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>


using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace FrameWrapper;


class ImageUndistortionTestInterface : public DFNTestInterface
	{
	public:
		ImageUndistortionTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~ImageUndistortionTestInterface();
	protected:

	private:
		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;
		Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubOutputCache;
		Mocks::MatToFrameConverter* mockOutputConverter;
		ImageUndistortion* undistort;

		cv::Mat cvImage;
		FrameConstPtr inputImage;
		std::string outputWindowName;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();
	};

ImageUndistortionTestInterface::ImageUndistortionTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight), inputImage()
	{
	undistort = new ImageUndistortion();
	SetDFN(undistort);

	MatToFrameConverter converter;
	cv::Mat doubleImage = cv::imread("../../tests/Data/Images/SmestechLab.jpg", cv::IMREAD_COLOR);
	cvImage = doubleImage(cv::Rect(doubleImage.cols/2,0,doubleImage.cols/2, doubleImage.rows));
	inputImage = converter.Convert(cvImage);
	undistort->imageInput(inputImage);
	outputWindowName = "Image Undistortion Result";
	}

ImageUndistortionTestInterface::~ImageUndistortionTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubOutputCache);
	delete(mockOutputConverter);
	delete(undistort);
	delete(inputImage);
	}

void ImageUndistortionTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubOutputCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	mockOutputConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubOutputCache, mockOutputConverter);
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
	}

void ImageUndistortionTestInterface::DisplayResult()
	{
	FrameConstPtr undistortedImage= undistort->filteredImageOutput();
	FrameToMatConverter converter;
	cv::Mat undistortedCvImage = converter.Convert(undistortedImage);

	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::imshow(outputWindowName, undistortedCvImage);
	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );

	delete(undistortedImage);
	}


int main(int argc, char** argv)
	{
	ImageUndistortionTestInterface interface("ImageUndistortion", 100, 40);
	interface.Run();
	};

/** @} */
