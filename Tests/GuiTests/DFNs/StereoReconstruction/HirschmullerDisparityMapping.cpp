/**
 * @author Alessandro Bianco
 */

/**
 * Test application for the DFN HirschmullerDisparityMapping
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <StereoReconstruction/HirschmullerDisparityMapping.hpp>
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
		HirschmullerDisparityMapping* disparityMapping;

		cv::Mat cvLeftImage;
		cv::Mat cvRightImage;
		std::string outputWindowName;

		void SetupParameters();
		void DisplayResult();
};

DisparityMappingTestInterface::DisparityMappingTestInterface(const std::string& dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
{
	disparityMapping = new HirschmullerDisparityMapping;
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
	AddParameter("Prefilter", "Maximum", 31, 255);

	AddParameter("Disparities", "Minimum", 0, 255);
	AddParameter("Disparities", "NumberOfIntervals", 128, 1024);
	AddParameter("Disparities", "UseMaximumDifference", 1, 1);
	AddParameter("Disparities", "MaximumDifference", 1, 255);
	AddParameter("Disparities", "SpeckleRange", 37, 255);
	AddParameter("Disparities", "SpeckleWindow", 92, 255);
	AddParameter("Disparities", "SmoothnessParameter1", 2400, 100000);
	AddParameter("Disparities", "SmoothnessParameter2", 10000, 100000);

	AddParameter("BlocksMatching", "BlockSize", 9, 255);
	AddParameter("BlocksMatching", "UniquenessRatio", 10, 255);

	AddParameter("GeneralParameters", "PointCloudSamplingDensity", 0.10, 1, 1e-5);
	AddParameter("GeneralParameters", "useFullScaleTwoPassAlgorithm", 0, 1);
	AddParameter("GeneralParameters", "useDisparityToDepthMap", 0, 1);

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

	AddParameter("ReconstructionSpace", "LimitX", 20, 100);
	AddParameter("ReconstructionSpace", "LimitY", 20, 100);
	AddParameter("ReconstructionSpace", "LimitZ", 10, 100);
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
