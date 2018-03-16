/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ScanlineOptimization.cpp
 * @date 15/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN Scanline Optimization.
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
#include <StereoReconstruction/ScanlineOptimization.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Mocks/Common/Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include <BaseTypes.hpp>
#include <Visualizers/PclVisualizer.hpp>


using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace FrameWrapper;
using namespace PointCloudWrapper;


class DisparityMappingTestInterface : public DFNTestInterface
	{
	public:
		DisparityMappingTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~DisparityMappingTestInterface();
	protected:

	private:
		Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr >* stubCloudCache;
		Mocks::PointCloudToPclPointCloudConverter* mockCloudConverter;

		Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>* stubInputCache;
		Mocks::PclPointCloudToPointCloudConverter* mockInputConverter;

		ScanlineOptimization* disparityMapping;

		cv::Mat cvLeftImage;
		cv::Mat cvRightImage;
		std::string outputWindowName;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();
	};

DisparityMappingTestInterface::DisparityMappingTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
	{
	disparityMapping = new ScanlineOptimization();
	SetDFN(disparityMapping);

	cv::Mat doubleImage = cv::imread("../../tests/Data/Images/chessboard9.jpg", cv::IMREAD_COLOR);
	cv::Mat cvLeftImage = doubleImage( cv::Rect(0,0,doubleImage.cols/2,doubleImage.rows) ); //cv::imread("../../tests/Data/Images/left06.jpg", cv::IMREAD_COLOR);
	cv::Mat cvRightImage = doubleImage( cv::Rect(doubleImage.cols/2,0,doubleImage.cols/2,doubleImage.rows) ); //cv::imread("../../tests/Data/Images/right06.jpg", cv::IMREAD_COLOR);

	MatToFrameConverter converter;
	FrameConstPtr leftFrame = converter.Convert(cvLeftImage);
	FrameConstPtr rightFrame = converter.Convert(cvRightImage);

	disparityMapping->leftImageInput(leftFrame);
	disparityMapping->rightImageInput(rightFrame);

	outputWindowName = "Disparity Mapping Result";
	Visualizers::PclVisualizer::Enable();
	}

DisparityMappingTestInterface::~DisparityMappingTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(disparityMapping);
	}

void DisparityMappingTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>();
	mockInputConverter = new Mocks::PclPointCloudToPointCloudConverter();
	ConversionCache<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr, PclPointCloudToPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	stubCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>();
	mockCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);
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

	AddParameter("CameraParameters", "PrinciplePointX", 341.73837439614, 1000, 1e-5);
	AddParameter("CameraParameters", "PrinciplePointY",  236.03250430604, 1000, 1e-5);
	AddParameter("CameraParameters", "FocalLength", 532.10224793655, 1000, 1e-5);
	AddParameter("CameraParameters", "Baseline", 0.012, 1, 1e-5);

	AddParameter("ReconstructionSpace", "LimitX", 20, 100);
	AddParameter("ReconstructionSpace", "LimitY", 20, 100);
	AddParameter("ReconstructionSpace", "LimitZ", 1, 100);
	}

void DisparityMappingTestInterface::DisplayResult()
	{
	PointCloudConstPtr pointCloud= disparityMapping->pointCloudOutput();

	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
	PRINT_TO_LOG("Number of points: ", GetNumberOfPoints(*pointCloud) );

	Visualizers::PclVisualizer::Enable();
	Visualizers::PclVisualizer::ShowPointCloud(pointCloud);

	delete(pointCloud);
	}


int main(int argc, char** argv)
	{
	DisparityMappingTestInterface interface("DisparityMapping", 100, 40);
	interface.Run();
	};

/** @} */
