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


using namespace dfn_ci;
using namespace Converters;
using namespace Common;


class HarrisDetector3DTestInterface : public DFNTestInterface
	{
	public:
		HarrisDetector3DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~HarrisDetector3DTestInterface();
	protected:

	private:
		Stubs::CacheHandler<CppTypes::PointCloud::ConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr >* stubInputCache;
		Mocks::PointCloudToPclPointCloudConverter* mockInputConverter;
		Stubs::CacheHandler<cv::Mat, CppTypes::VisualPointFeatureVector3D::ConstPtr>* stubOutputCache;
		Mocks::MatToVisualPointFeatureVector3DConverter* mockOutputConverter;
		HarrisDetector3D* harris;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
		CppTypes::PointCloud::ConstPtr inputCloud;
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
	pclCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::io::loadPLYFile("../tests/Data/PointClouds/bunny0.ply", *pclCloud);
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
	inputCloud.reset();
	}

void HarrisDetector3DTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<CppTypes::PointCloud::ConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>();
	mockInputConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<CppTypes::PointCloud::ConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	stubOutputCache = new Stubs::CacheHandler<cv::Mat, CppTypes::VisualPointFeatureVector3D::ConstPtr>();
	mockOutputConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, CppTypes::VisualPointFeatureVector3D::ConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubOutputCache, mockOutputConverter);
	}

void HarrisDetector3DTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "NonMaxSuppression", 1, 1);
	AddParameter("GeneralParameters", "Radius", 0.02, 1.00, 0.01);
	AddParameter("GeneralParameters", "EnableRefinement", 0, 1);
	AddParameter("GeneralParameters", "DetectionThreshold", 0.50, 1.00, 0.01);
	AddParameter("GeneralParameters", "NumberOfThreads", 0, 10);
	AddParameter("GeneralParameters", "HarrisMethod", 0, 4);
	}

void HarrisDetector3DTestInterface::DisplayResult()
	{
	CppTypes::VisualPointFeatureVector3D::ConstPtr featuresVector= harris->featuresSetOutput();

	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );

	pcl::PointCloud<pcl::PointXYZ>::Ptr featuresCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>() );
	for(int pointIndex = 0; pointIndex < featuresVector->GetNumberOfPoints(); pointIndex++)
		{
		pcl::PointXYZ newPoint(featuresVector->GetXCoordinate(pointIndex), featuresVector->GetYCoordinate(pointIndex), featuresVector->GetZCoordinate(pointIndex) );
		featuresCloud->points.push_back(newPoint);
		}

	pcl::visualization::PCLVisualizer viewer (outputWindowName);
    	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclCloudColor(pclCloud, 255, 255, 255);
    	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> featuresCloudColor(featuresCloud, 255, 0, 0);
    	viewer.addPointCloud(pclCloud,pclCloudColor,"input");
    	viewer.addPointCloud(featuresCloud,featuresCloudColor,"keypoints");

    	while (!viewer.wasStopped ())
    		{
        	viewer.spinOnce();
        	pcl_sleep (0.01);
    		} 

	featuresVector.reset();
	}


int main(int argc, char** argv)
	{
	HarrisDetector3DTestInterface interface("HarrisDetector3D", 100, 40);
	interface.Run();
	};

/** @} */
