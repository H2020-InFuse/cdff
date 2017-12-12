/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisDetector2D.cpp
 * @date 23/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN HarrisDetector2D.
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
#include <FeaturesExtraction2D/HarrisDetector2D.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>


using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace CppTypes;


class HarrisDetector2DTestInterface : public DFNTestInterface
	{
	public:
		HarrisDetector2DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~HarrisDetector2DTestInterface();
	protected:

	private:
		Stubs::CacheHandler<Frame::ConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;
		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2D::ConstPtr>* stubOutputCache;
		Mocks::MatToVisualPointFeatureVector2DConverter* mockOutputConverter;
		HarrisDetector2D* harris;

		cv::Mat cvImage;
		Frame::ConstPtr inputImage;
		std::string outputWindowName;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();
	};

HarrisDetector2DTestInterface::HarrisDetector2DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight), inputImage()
	{
	harris = new HarrisDetector2D();
	SetDFN(harris);

	MatToFrameConverter converter;
	cvImage = cv::imread("../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);
	inputImage = converter.Convert(cvImage);
	harris->imageInput(inputImage);

	outputWindowName = "Harris Detector 2D Result";
	}

HarrisDetector2DTestInterface::~HarrisDetector2DTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubOutputCache);
	delete(mockOutputConverter);
	delete(harris);
	inputImage.reset();
	}

void HarrisDetector2DTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<Frame::ConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<Frame::ConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubOutputCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2D::ConstPtr>();
	mockOutputConverter = new Mocks::MatToVisualPointFeatureVector2DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector2D::ConstPtr, MatToVisualPointFeatureVector2DConverter>::Instance(stubOutputCache, mockOutputConverter);
	}

void HarrisDetector2DTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "ApertureSize", 5, 7);
	AddParameter("GeneralParameters", "BlockSize", 2, 50);
	AddParameter("GeneralParameters", "ParameterK", 0.04, 1.00, 0.01);
	AddParameter("GeneralParameters", "DetectionThreshold", 200, 255);
	AddParameter("GeneralParameters", "UseGaussianBlur", 0, 1);
	AddParameter("GaussianBlur", "KernelWidth", 3, 99);
	AddParameter("GaussianBlur", "KernelHeight", 3, 99);
	AddParameter("GaussianBlur", "WidthStandardDeviation", 0.00, 1.00, 0.01);
	AddParameter("GaussianBlur", "HeightStandardDeviation", 0.00, 1.00, 0.01);
	}

void HarrisDetector2DTestInterface::DisplayResult()
	{
	VisualPointFeatureVector2D::ConstPtr featuresVector= harris->featuresSetOutput();
	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::Mat outputImage = cvImage.clone();

	for(int featureIndex = 0; featureIndex < featuresVector->GetNumberOfPoints(); featureIndex++)
		{
		cv::Point drawPoint(featuresVector->GetXCoordinate(featureIndex), featuresVector->GetYCoordinate(featureIndex) );
		cv::circle(outputImage, drawPoint, 5, cv::Scalar(0, 0, 255), 2, 8, 0);
		}

	cv::imshow(outputWindowName, outputImage);
	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );

	featuresVector.reset();
	}


int main(int argc, char** argv)
	{
	HarrisDetector2DTestInterface interface("HarrisDetector2D", 100, 40);
	interface.Run();
	};

/** @} */
