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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <FeaturesExtraction2D/HarrisDetector2D.hpp>
#include <MatToFrameConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>


using namespace dfn_ci;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace FrameWrapper;


class HarrisDetector2DTestInterface : public DFNTestInterface
	{
	public:
		HarrisDetector2DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight, std::string imageFilePath = DEFAULT_IMAGE_FILE_PATH);
		~HarrisDetector2DTestInterface();
	protected:

	private:
		static const std::string DEFAULT_IMAGE_FILE_PATH;
		HarrisDetector2D* harris;

		cv::Mat cvImage;
		FrameConstPtr inputImage;
		std::string outputWindowName;

		void SetupParameters();
		void DisplayResult();
	};

const std::string HarrisDetector2DTestInterface::DEFAULT_IMAGE_FILE_PATH = "../../tests/Data/Images/DevonIslandLeft.ppm";

HarrisDetector2DTestInterface::HarrisDetector2DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight, std::string imageFilePath)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight), inputImage()
	{
	harris = new HarrisDetector2D();
	SetDFN(harris);

	cvImage = cv::imread(imageFilePath, cv::IMREAD_COLOR);
	inputImage = MatToFrameConverter().Convert(cvImage);

	harris->frameInput(*inputImage);
	outputWindowName = "Harris Detector 2D Result";
	}

HarrisDetector2DTestInterface::~HarrisDetector2DTestInterface()
	{
	delete(harris);
	delete(inputImage);
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
	const VisualPointFeatureVector2D& featuresVector = harris->featuresOutput();

	PRINT_TO_LOG("Number of detected features: ", GetNumberOfPoints( featuresVector ) );
	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::Mat outputImage = cvImage.clone();

	for (int featureIndex = 0; featureIndex < GetNumberOfPoints(featuresVector); featureIndex++)
		{
		cv::Point drawPoint(GetXCoordinate(featuresVector, featureIndex), GetYCoordinate(featuresVector, featureIndex) );
		cv::circle(outputImage, drawPoint, 5, cv::Scalar(0, 0, 255), 2, 8, 0);
		}

	cv::imshow(outputWindowName, outputImage);

	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
	}

int main(int argc, char** argv)
	{
	HarrisDetector2DTestInterface* interface;
	if (argc > 1)
		{
		interface = new HarrisDetector2DTestInterface("HarrisDetector2D", 100, 40, argv[1]);
		}
	else
		{
		interface = new HarrisDetector2DTestInterface("HarrisDetector2D", 100, 40);
		}
	interface->Run();
	delete(interface);
	};

/** @} */
