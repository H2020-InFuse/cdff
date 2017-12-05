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
#include <PointCloud3DToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloud3DConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/PointCloud3DToPclPointCloudConverter.hpp>
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
		Stubs::CacheHandler<PointCloud3D*, pcl::PointCloud<pcl::PointXYZ>::Ptr >* stubInputCache;
		Mocks::PointCloud3DToPclPointCloudConverter* mockInputConverter;
		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3D*>* stubOutputCache;
		Mocks::MatToVisualPointFeatureVector3DConverter* mockOutputConverter;
		HarrisDetector3D* harris;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
		PointCloud3D* inputCloud;
		VisualPointFeatureVector3D featuresVector;
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

	PclPointCloudToPointCloud3DConverter converter;
	pclCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::io::loadPLYFile("", *pclCloud);
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
	stubInputCache = new Stubs::CacheHandler<PointCloud3D*, pcl::PointCloud<pcl::PointXYZ>::Ptr>();
	mockInputConverter = new Mocks::PointCloud3DToPclPointCloudConverter();
	ConversionCache<PointCloud3D*, pcl::PointCloud<pcl::PointXYZ>::Ptr, PointCloud3DToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	stubOutputCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3D*>();
	mockOutputConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector3D*, MatToVisualPointFeatureVector3DConverter>::Instance(stubOutputCache, mockOutputConverter);

	
	mockInputConverter->AddBehaviour("Convert", "Always", (void*) (&pclCloud) );
	//mockOutputConverter->AddBehaviour("Convert", "Always", (void*) (&featuresVector) );
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
	VisualPointFeatureVector3D* featuresVector= harris->featuresSetOutput();

	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );

	pcl::PointCloud<pcl::PointXYZ>::Ptr featuresCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>() );
	for(int pointIndex = 0; pointIndex < featuresVector->list.count; pointIndex++)
		{
		VisualPointFeature3D* feature = featuresVector->list.array[pointIndex];
		pcl::PointXYZ newPoint(feature->point.x, feature->point.y, feature->point.z);
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

	//The cache should handle this cancellation, but we only have a stub cache at the moment 
	asn_sequence_empty( &(featuresVector->list) );
	delete(featuresVector);
	}


int main(int argc, char** argv)
	{
	HarrisDetector3DTestInterface interface("HarrisDetector3D", 100, 40);
	interface.Run();
	};

/** @} */
