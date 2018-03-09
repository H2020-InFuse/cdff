/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DisparityMapping.cpp
 * @date 09/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN Disparity Mapping.
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
#include <StereoReconstruction/DisparityMapping.hpp>
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

		DisparityMapping* disparityMapping;

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
	disparityMapping = new DisparityMapping();
	SetDFN(disparityMapping);

	cv::Mat cvImage = cv::imread("../../tests/Data/Images/SmestechLab.jpg", cv::IMREAD_COLOR);
	cvLeftImage = cvImage( cv::Rect(0, 0, cvImage.cols/2, cvImage.rows) );
	cvRightImage = cvImage( cv::Rect(cvImage.cols/2, 0, cvImage.cols/2, cvImage.rows) );

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
	AddSignedParameter("DisparityToDepthMap", "Element_0_3", -3259.404846191406, 4000, 1e-5);
	AddParameter("DisparityToDepthMap", "Element_1_0", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_1_1", 1, 255);
	AddParameter("DisparityToDepthMap", "Element_1_2", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_1_3", 1793.122953414917, 2000, 1e-5);
	AddParameter("DisparityToDepthMap", "Element_2_0", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_2_1", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_2_2", 0, 255);
	AddSignedParameter("DisparityToDepthMap", "Element_2_3", -1822.793404684805, 2000, 1e-5);
	AddParameter("DisparityToDepthMap", "Element_3_0", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_3_1", 0, 255);
	AddParameter("DisparityToDepthMap", "Element_3_2", 0.3911752722551165, 1, 1e-7);
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
