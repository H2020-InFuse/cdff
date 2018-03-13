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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <ImageFiltering/ImageUndistortionRectification.hpp>
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


class ImageUndistortionRectificationTestInterface : public DFNTestInterface
	{
	public:
		ImageUndistortionRectificationTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~ImageUndistortionRectificationTestInterface();
	protected:

	private:
		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;
		Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubOutputCache;
		Mocks::MatToFrameConverter* mockOutputConverter;
		ImageUndistortionRectification* undistort;

		cv::Mat cvImage;
		FrameConstPtr inputImage;
		std::string outputWindowName;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();
	};

ImageUndistortionRectificationTestInterface::ImageUndistortionRectificationTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight), inputImage()
	{
	undistort = new ImageUndistortionRectification();
	SetDFN(undistort);

	MatToFrameConverter converter;
	cv::Mat doubleImage = cv::imread("../../tests/Data/Images/SmestechLab.jpg", cv::IMREAD_COLOR);
	cvImage = doubleImage(cv::Rect(doubleImage.cols/2,0,doubleImage.cols/2, doubleImage.rows));
	inputImage = converter.Convert(cvImage);
	undistort->imageInput(inputImage);
	outputWindowName = "Image Undistortion Rectification Result";
	}

ImageUndistortionRectificationTestInterface::~ImageUndistortionRectificationTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubOutputCache);
	delete(mockOutputConverter);
	delete(undistort);
	delete(inputImage);
	}

void ImageUndistortionRectificationTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubOutputCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	mockOutputConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubOutputCache, mockOutputConverter);
	}

void ImageUndistortionRectificationTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "ConstantBorderValue", 0, 100, 1e-2);
	AddParameter("GeneralParameters", "InterpolationMethod", 0, 3);
	AddParameter("GeneralParameters", "BorderMode", 0, 2);
	}

void ImageUndistortionRectificationTestInterface::DisplayResult()
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
	ImageUndistortionRectificationTestInterface interface("ImageUndistortionRectification", 100, 40);
	interface.Run();
	};

/** @} */
