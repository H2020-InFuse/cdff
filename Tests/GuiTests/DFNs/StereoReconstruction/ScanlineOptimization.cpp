/**
 * @author Alessandro Bianco
 */

/**
 * Test application for the DFN ScanlineOptimization
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <StereoReconstruction/ScanlineOptimization.hpp>
#include <MatToFrameConverter.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include <Visualizers/PclVisualizer.hpp>
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
		DisparityMappingTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~DisparityMappingTestInterface();

	private:
		ScanlineOptimization* disparityMapping;

		cv::Mat cvLeftImage;
		cv::Mat cvRightImage;
		std::string outputWindowName;

		void SetupParameters();
		void DisplayResult();
};

DisparityMappingTestInterface::DisparityMappingTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
{
	disparityMapping = new ScanlineOptimization;
	SetDFN(disparityMapping);

	cvLeftImage = cv::imread("../../tests/Data/Images/RectifiedChair40Left.png", cv::IMREAD_COLOR);
	cvRightImage = cv::imread("../../tests/Data/Images/RectifiedChair40Right.png", cv::IMREAD_COLOR);

	MatToFrameConverter matToFrame;
	const Frame* left = matToFrame.Convert(cvLeftImage);
	const Frame* right = matToFrame.Convert(cvRightImage);

	disparityMapping->leftInput(*left);
	disparityMapping->rightInput(*right);

	outputWindowName = "Disparity Mapping Result";
	Visualizers::PclVisualizer::Enable();
}

DisparityMappingTestInterface::~DisparityMappingTestInterface()
{
	delete disparityMapping;
}

void DisparityMappingTestInterface::SetupParameters()
{
		struct CameraParameters
		{
			float principlePointX;
			float principlePointY;
			float focalLength;
			float baseline;
		};

		struct MatchingOptionsSet
		{
			int numberOfDisparities;
			int horizontalOffset;
			int ratioFilter;
			int peakFilter;
			bool usePreprocessing;
			bool useLeftRightConsistencyCheck;
			int leftRightConsistencyThreshold;
		};

		struct ScanlineOptimizationOptionsSet
		{
			int costAggregationRadius;
			int spatialBandwidth;
			int colorBandwidth;
			int strongSmoothnessPenalty;
			int weakSmoothnessPenalty;
			MatchingOptionsSet matchingOptionsSet;
			CameraParameters cameraParameters;
		};

	AddParameter("GeneralParameters", "CostAggregationRadius", 5, 255);
	AddParameter("GeneralParameters", "SpatialBandwidth", 25, 255);
	AddParameter("GeneralParameters", "ColorBandwidth", 15, 255);
	AddParameter("GeneralParameters", "StrongSmoothnessPenalty", 1, 255);
	AddParameter("GeneralParameters", "WeakSmoothnessPenalty", 1, 255);
	AddParameter("GeneralParameters", "PointCloudSamplingDensity", 0.1, 1, 0.1);

	AddParameter("Matching", "NumberOfDisparities", 60, 255);
	AddParameter("Matching", "HorizontalOffset", 1, 255);
	AddParameter("Matching", "RatioFilter", 5, 255);
	AddParameter("Matching", "PeakFilter", 1, 255);
	AddParameter("Matching", "UsePreprocessing", 0, 1);
	AddParameter("Matching", "UseLeftRightConsistencyCheck", 0, 1);
	AddParameter("Matching", "LeftRightConsistencyThreshold", 1, 255);

	AddParameter("StereoCamera", "LeftFocalLength", 693.4181807813, 700, 1e-5);
	AddParameter("StereoCamera", "LeftPrinciplePointX", 671.7716154809, 700, 1e-5);
	AddParameter("StereoCamera", "LeftPrinciplePointY", 391.33378485796, 700, 1e-5);
	AddParameter("StereoCamera", "Baseline", 0.012, 1, 1e-5);

	AddParameter("ReconstructionSpace", "LimitX", 20, 100);
	AddParameter("ReconstructionSpace", "LimitY", 20, 100);
	AddParameter("ReconstructionSpace", "LimitZ", 1, 100);
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
