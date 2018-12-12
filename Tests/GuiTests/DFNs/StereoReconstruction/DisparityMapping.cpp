/**
 * @author Alessandro Bianco
 */

/**
 * Test application for the DFN DisparityMapping
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <StereoReconstruction/DisparityMapping.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include <Visualizers/OpenCVVisualizer.hpp>
#include <Visualizers/PCLVisualizer.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace CDFF::DFN::StereoReconstruction;
using namespace Converters;
using namespace FrameWrapper;
using namespace PointCloudWrapper;

class DisparityMappingTestInterface : public DFNTestInterface
{
	public:
		DisparityMappingTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight);
		~DisparityMappingTestInterface();

	private:
		DisparityMapping* disparityMapping;

		cv::Mat cvLeftImage;
		cv::Mat cvRightImage;
		std::string outputWindowName;

		void SetupParameters();
		void DisplayResult();
};

DisparityMappingTestInterface::DisparityMappingTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
{
	disparityMapping = new DisparityMapping;
	SetDFN(disparityMapping);

	cvLeftImage = cv::imread("../../tests/Data/Images/RectifiedChair40Left.png", cv::IMREAD_COLOR);
	cvRightImage = cv::imread("../../tests/Data/Images/RectifiedChair40Right.png", cv::IMREAD_COLOR);

	MatToFrameConverter matToFrame;
	const Frame* left = matToFrame.Convert(cvLeftImage);
	const Frame* right = matToFrame.Convert(cvRightImage);

	disparityMapping->leftInput(*left);
	disparityMapping->rightInput(*right);

	outputWindowName = "Disparity Mapping Result";
	Visualizers::OpencvVisualizer::Enable();
}

DisparityMappingTestInterface::~DisparityMappingTestInterface()
{
	delete disparityMapping;
}

void DisparityMappingTestInterface::SetupParameters()
{
	AddParameter("Prefilter", "Size", 9, 255);
	AddParameter("Prefilter", "Type", 1, 2);
	AddParameter("Prefilter", "Maximum", 31, 255);

	AddParameter("Disparities", "Minimum", 0, 255);
	AddParameter("Disparities", "NumberOfIntervals", 64, 1024);
	AddParameter("Disparities", "UseMaximumDifference", 0, 1);
	AddParameter("Disparities", "MaximumDifference", 100, 255);
	AddParameter("Disparities", "SpeckleRange", 0, 255);
	AddParameter("Disparities", "SpeckleWindowSize", 0, 255);

	AddParameter("BlocksMatching", "BlockSize", 21, 255);
	AddParameter("BlocksMatching", "SmallerBlockSize", 0, 255);
	AddParameter("BlocksMatching", "UniquenessRatio", 10, 255);
	AddParameter("BlocksMatching", "TextureThreshold", 15, 255);

	AddParameter("FirstRegionOfInterest", "TopLefColumn", 0, 255);
	AddParameter("FirstRegionOfInterest", "TopLeftRow", 0, 255);
	AddParameter("FirstRegionOfInterest", "NumberOfColumns", 0, 255);
	AddParameter("FirstRegionOfInterest", "NumberOfRows", 0, 255);

	AddParameter("SecondRegionOfInterest", "TopLefColumn", 0, 255);
	AddParameter("SecondRegionOfInterest", "TopLeftRow", 0, 255);
	AddParameter("SecondRegionOfInterest", "NumberOfColumns", 0, 255);
	AddParameter("SecondRegionOfInterest", "NumberOfRows", 0, 255);

	AddParameter("GeneralParameters", "PointCloudSamplingDensity", 0.10, 1, 1e-5);

	AddParameter("DisparityToDepthMap", "Element_0_0", 1, 255);
	AddParameter("DisparityToDepthMap", "Element_0_1", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_0_2", 0, 255);
	AddSignedParameter("DisparityToDepthMap", "Element_0_3", -279.0498046875, 2000, 1e-5);
	AddParameter("DisparityToDepthMap", "Element_1_0", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_1_1", 1, 255);
	AddParameter("DisparityToDepthMap", "Element_1_2", 0, 255);
	AddSignedParameter("DisparityToDepthMap", "Element_1_3",29.868621826172, 2000, 1e-5);
	AddParameter("DisparityToDepthMap", "Element_2_0", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_2_1", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_2_2", 0, 255);
	AddSignedParameter("DisparityToDepthMap", "Element_2_3", -8192.8300337838, 10000, 1e-4);
	AddParameter("DisparityToDepthMap", "Element_3_0", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_3_1", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_3_2", 3.3436329786051, 10, 1e-6);
	AddParameter("DisparityToDepthMap", "Element_3_3", 0, 255);

	AddParameter("StereoCamera", "LeftFocalLength", 693.4181807813, 700, 1e-5);
	AddParameter("StereoCamera", "LeftPrinciplePointX", 671.7716154809, 700, 1e-5);
	AddParameter("StereoCamera", "LeftPrinciplePointY", 391.33378485796, 700, 1e-5);
	AddParameter("StereoCamera", "Baseline", 0.012, 1, 1e-5);
}

void DisparityMappingTestInterface::DisplayResult()
{
	const PointCloud& pointcloud = disparityMapping->pointcloudOutput();

	PRINT_TO_LOG("Processing time (seconds): ", GetLastProcessingTimeSeconds());
	PRINT_TO_LOG("Virtual memory used (kB): ", GetTotalVirtualMemoryUsedKB());
	PRINT_TO_LOG("Number of points: ", GetNumberOfPoints(pointcloud));

	Visualizers::PclVisualizer::Enable();
	Visualizers::PclVisualizer::ShowPointCloud(&pointcloud);
}

int main(int argc, char** argv)
{
	DisparityMappingTestInterface interface("DisparityMapping", 100, 40);
	interface.Run();
};

/** @} */
