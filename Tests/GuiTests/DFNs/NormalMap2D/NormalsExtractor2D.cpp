// includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <NormalMap2D/NormalsExtractor2D.hpp>
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


class NormalsExtractor2DTestInterface : public DFNTestInterface
	{
	public:
		NormalsExtractor2DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~NormalsExtractor2DTestInterface();
	protected:

	private:
		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;
		Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubOutputCache;
		Mocks::MatToFrameConverter* mockOutputConverter;
		NormalsExtractor2D* normals;

		cv::Mat cvImage;
		FrameConstPtr depthImage;
		std::string outputWindowName;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();
	};

NormalsExtractor2DTestInterface::NormalsExtractor2DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight), depthImage()
	{
	normals = new NormalsExtractor2D();
	SetDFN(normals);

	MatToFrameConverter converter;
	cvImage = cv::imread("../../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);
	depthImage = converter.Convert(cvImage);
	normals->imageInput(depthImage);
	outputWindowName = "NormalsExtractor 2D Result";
	}

NormalsExtractor2DTestInterface::~NormalsExtractor2DTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubOutputCache);
	delete(mockOutputConverter);
	delete(normals);
	delete(depthImage);
	}

void NormalsExtractor2DTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubOutputCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	mockOutputConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubOutputCache, mockOutputConverter);
	}

void NormalsExtractor2DTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "scale", 1, 1);
	}

void NormalsExtractor2DTestInterface::DisplayResult()
	{
	FrameConstPtr Normals = normals->NormalMapOutput();
	cv::namedWindow(outputWindowName, CV_WINDOW_NORMAL);
	cv::Mat outputImage = cvImage.clone();

	cv::imshow(outputWindowName, outputImage);
	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );

	delete(Normals);
	}


int main(int argc, char** argv)
	{
	NormalsExtractor2DTestInterface interface("NormalsExtractor2D", 100, 40); // correspond to button width + height
	interface.Run();
	};

/** @} */
