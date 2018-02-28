/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisDetector3D.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN HarrisDetector3D.
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
#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include<pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Visualizers/PclVisualizer.hpp>


using namespace dfn_ci;
using namespace Converters;
using namespace Common;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;

class HarrisDetector3DTestInterface : public DFNTestInterface
	{
	public:
		HarrisDetector3DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~HarrisDetector3DTestInterface();
	protected:

	private:
		Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr >* stubInputCache;
		Mocks::PointCloudToPclPointCloudConverter* mockInputConverter;
		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>* stubOutputCache;
		Mocks::MatToVisualPointFeatureVector3DConverter* mockOutputConverter;
		HarrisDetector3D* harris;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
		PointCloudConstPtr inputCloud;
		std::string outputWindowName;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();
	};

HarrisDetector3DTestInterface::HarrisDetector3DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
	{
	harris = new HarrisDetector3D();
	SetDFN(harris);

	PclPointCloudToPointCloudConverter converter;
	pcl::PointCloud<pcl::PointXYZ>::Ptr basePclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile("../../tests/Data/PointClouds/bunny0.ply", *basePclCloud);
	for(unsigned pointIndex = 0; pointIndex < basePclCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = basePclCloud->points.at(pointIndex);
		if (point.x == point.x && point.y == point.y && point.z == point.z)
			{
			pclCloud->points.push_back(point);
			}
		}
	inputCloud = converter.Convert(pclCloud);
	harris->pointCloudInput(inputCloud);
	outputWindowName = "Harris Detector 3D Result";
	}

HarrisDetector3DTestInterface::~HarrisDetector3DTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubOutputCache);
	delete(mockOutputConverter);
	delete(harris);
	delete(inputCloud);
	}

void HarrisDetector3DTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>();
	mockInputConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	stubOutputCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	mockOutputConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubOutputCache, mockOutputConverter);
	}

void HarrisDetector3DTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "NonMaxSuppression", 1, 1);
	AddParameter("GeneralParameters", "Radius", 0.010, 0.100, 0.001);
	AddParameter("GeneralParameters", "SearchRadius", 0.010, 0.100, 0.001);
	AddParameter("GeneralParameters", "EnableRefinement", 0, 1);
	AddParameter("GeneralParameters", "DetectionThreshold", 0.0010, 0.0100, 0.0001);
	AddParameter("GeneralParameters", "NumberOfThreads", 0, 10);
	AddParameter("GeneralParameters", "HarrisMethod", 0, 4);
	}

void HarrisDetector3DTestInterface::DisplayResult()
	{
	VisualPointFeatureVector3DConstPtr featuresVector= harris->featuresSetOutput();

	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
	PRINT_TO_LOG("Number of feature points: ", GetNumberOfPoints(*featuresVector) );

	Visualizers::PclVisualizer::Enable();
	Visualizers::PclVisualizer::ShowVisualFeatures(inputCloud, featuresVector);

	delete(featuresVector);
	}


int main(int argc, char** argv)
	{
	HarrisDetector3DTestInterface interface("HarrisDetector3D", 100, 40);
	interface.Run();
	};

/** @} */
