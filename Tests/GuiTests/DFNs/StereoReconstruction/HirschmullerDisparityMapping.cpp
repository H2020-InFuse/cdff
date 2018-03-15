/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HirschmullerDisparityMapping.cpp
 * @date 15/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN Hirschmuller Disparity Mapping.
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
#include <StereoReconstruction/HirschmullerDisparityMapping.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
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

		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubInputCache;
		Mocks::FrameToMatConverter* mockInputConverter;

		HirschmullerDisparityMapping* disparityMapping;

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
	disparityMapping = new HirschmullerDisparityMapping();
	SetDFN(disparityMapping);

	cv::Mat cvLeftImage = cv::imread("../../tests/Data/Images/RectifiedLeft.jpg", cv::IMREAD_COLOR);
	cv::Mat cvRightImage = cv::imread("../../tests/Data/Images/RectifiedRight.jpg", cv::IMREAD_COLOR);

	MatToFrameConverter converter;
	FrameConstPtr leftFrame = converter.Convert(cvLeftImage);
	FrameConstPtr rightFrame = converter.Convert(cvRightImage);

	disparityMapping->leftImageInput(leftFrame);
	disparityMapping->rightImageInput(rightFrame);

	outputWindowName = "Disparity Mapping Result";
	}

DisparityMappingTestInterface::~DisparityMappingTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(disparityMapping);
	}

void DisparityMappingTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockInputConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInputCache, mockInputConverter);

	stubCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>();
	mockCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);
	}

void DisparityMappingTestInterface::SetupParameters()
	{
	AddParameter("Prefilter", "Maximum", 31, 255);

	AddParameter("Disparities", "Minimum", 0, 255);
	AddParameter("Disparities", "NumberOfIntervals", 128, 1024);
	AddParameter("Disparities", "UseMaximumDifference", 1, 1);
	AddParameter("Disparities", "MaximumDifference", 1, 255);
	AddParameter("Disparities", "SpeckleRange", 37, 255);
	AddParameter("Disparities", "SpeckleWindowSize", 92, 255);
	AddParameter("Disparities", "SmoothnessParameter1", 2400, 100000);
	AddParameter("Disparities", "SmoothnessParameter2", 10000, 100000);

	AddParameter("BlocksMatching", "BlockSize", 9, 255);
	AddParameter("BlocksMatching", "UniquenessRatio", 10, 255);

	AddParameter("GeneralParameters", "PointCloudSamplingDensity", 0.10, 1, 1e-5);
	AddParameter("GeneralParameters", "useFullScaleTwoPassAlgorithm", 0, 1);

	AddParameter("DisparityToDepthMap", "Element_0_0", 1, 255);
	AddParameter("DisparityToDepthMap", "Element_0_1", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_0_2", 0, 255);
	AddSignedParameter("DisparityToDepthMap", "Element_0_3", -270.37537384033, 2000, 1e-5);
	AddParameter("DisparityToDepthMap", "Element_1_0", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_1_1", 1, 255);
	AddParameter("DisparityToDepthMap", "Element_1_2", 0, 255);
	AddSignedParameter("DisparityToDepthMap", "Element_1_3",-229.79105377197, 2000, 1e-5);
	AddParameter("DisparityToDepthMap", "Element_2_0", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_2_1", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_2_2", 0, 255);
	AddSignedParameter("DisparityToDepthMap", "Element_2_3", -1121.1182691834, 2000, 1e-5);
	AddParameter("DisparityToDepthMap", "Element_3_0", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_3_1", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_3_2", 6.0102127133098, 10, 1e-6);
	AddParameter("DisparityToDepthMap", "Element_3_3", 0, 255);
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
