// includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <KMeans2D/Kmeans.hpp>
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


class KmeansTestInterface : public DFNTestInterface
	{
	public:
		KmeansTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~KmeansTestInterface();
	protected:

	private:
		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;
		Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubOutputCache;
		Mocks::MatToFrameConverter* mockOutputConverter;
		Kmeans* kmeans;

		cv::Mat cvImage;
		FrameConstPtr inputImage;
		std::string outputWindowName;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();
	};

KmeansTestInterface::KmeansTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight), inputImage()
	{
	kmeans = new Kmeans();
	SetDFN(kmeans);

	MatToFrameConverter converter;
	cvImage = cv::imread("../../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);
	inputImage = converter.Convert(cvImage);
	kmeans->imageInput(inputImage);
	outputWindowName = "Kmeans 2DResult";
	}

KmeansTestInterface::~KmeansTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubOutputCache);
	delete(mockOutputConverter);
	delete(kmeans);
	delete(inputImage);
	}

void KmeansTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubOutputCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	mockOutputConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubOutputCache, mockOutputConverter);
	}

void KmeansTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "nbZonesK", 5, 7);
	}

void KmeansTestInterface::DisplayResult()
	{
	FrameConstPtr CannyEdge = kmeans->KMeansImageOutput();
	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::Mat outputImage = cvImage.clone();

	cv::imshow(outputWindowName, outputImage);
	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );

	delete(CannyEdge);
	}


int main(int argc, char** argv)
	{
	KmeansTestInterface interface("Kmeans", 100, 40); // correspond to button width + height
	interface.Run();
	};

/** @} */
