// includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <EdgeMap2D/CannyEdgeExtractor2D.hpp>
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


class CannyEdgeExtractor2DTestInterface : public DFNTestInterface
	{
	public:
		CannyEdgeExtractor2DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~CannyEdgeExtractor2DTestInterface();
	protected:

	private:
		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;
		Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubOutputCache;
		Mocks::MatToFrameConverter* mockOutputConverter;
		CannyEdgeExtractor2D* canny;

		cv::Mat cvImage;
		FrameConstPtr inputImage;
		std::string outputWindowName;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();
	};

CannyEdgeExtractor2DTestInterface::CannyEdgeExtractor2DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight), inputImage()
	{
	canny = new CannyEdgeExtractor2D();
	SetDFN(canny);

	MatToFrameConverter converter;
	cvImage = cv::imread("../../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);
	inputImage = converter.Convert(cvImage);
	canny->imageInput(inputImage);
	outputWindowName = "CannyEdgeExtractor 2D Result";
	}

CannyEdgeExtractor2DTestInterface::~CannyEdgeExtractor2DTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubOutputCache);
	delete(mockOutputConverter);
	delete(canny);
	delete(inputImage);
	}

void CannyEdgeExtractor2DTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubOutputCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	mockOutputConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubOutputCache, mockOutputConverter);
	}

void CannyEdgeExtractor2DTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "kernelSize", 5, 7);
	AddParameter("GeneralParameters", "threshold", 50, 100);
	}

void CannyEdgeExtractor2DTestInterface::DisplayResult()
	{
	FrameConstPtr CannyEdge = canny->EdgeMapOutput();
	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::Mat outputImage = cvImage.clone();

	cv::imshow(outputWindowName, outputImage);
	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );

	delete(CannyEdge);
	}


int main(int argc, char** argv)
	{
	CannyEdgeExtractor2DTestInterface interface("CannyEdgeExtractor2D", 100, 40); // correspond to button width + height
	interface.Run();
	};

/** @} */
