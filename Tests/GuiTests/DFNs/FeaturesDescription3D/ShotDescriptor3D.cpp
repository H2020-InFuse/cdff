/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ShotDescriptor3D.cpp
 * @date 24/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN ShotDescriptor3D.
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
#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <PclNormalsCloudToPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclNormalsCloudConverter.hpp>
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
using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;

class ShotDescriptor3DTestInterface : public DFNTestInterface
	{
	public:
		ShotDescriptor3DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~ShotDescriptor3DTestInterface();
	protected:

	private:
		static const unsigned SHOT_DESCRIPTOR_SIZE;

		Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr >* stubInputCache;
		Mocks::PointCloudToPclPointCloudConverter* mockInputConverter;
		Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr >* stubInputNormalsCache;
		Mocks::PointCloudToPclNormalsCloudConverter* mockInputNormalsConverter;
		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>* stubOutputCache;
		Mocks::MatToVisualPointFeatureVector3DConverter* mockOutputConverter;
		ShotDescriptor3D* shot;

		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
		PointCloudConstPtr inputCloud;
		VisualPointFeatureVector3DConstPtr inputFeaturesSet;
		pcl::PointCloud<pcl::Normal>::Ptr pclNormalsCloud;
		PointCloudConstPtr inputNormalsCloud;
		std::string outputWindowName;

		pcl::PointCloud<pcl::PointXYZ>::Ptr PreparePointCloudInput();

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr PrepareOutputCloud(VisualPointFeatureVector3DConstPtr featuresVector);
		void GetComponentRange(VisualPointFeatureVector3DConstPtr featuresVector, int componentIndex, float& min, float& max);
	};

const unsigned ShotDescriptor3DTestInterface::SHOT_DESCRIPTOR_SIZE = 352;

ShotDescriptor3DTestInterface::ShotDescriptor3DTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
	{
	shot = new ShotDescriptor3D();
	SetDFN(shot);

	PclPointCloudToPointCloudConverter converter;
	pclCloud = PreparePointCloudInput();
	inputCloud = converter.Convert(pclCloud);
	shot->pointCloudInput(inputCloud);

	VisualPointFeatureVector3DPtr featuresSet = NewVisualPointFeatureVector3D();
	ClearPoints(*featuresSet);
	for(unsigned index = 20; index < pclCloud->points.size(); index += 20)
		{
		AddPoint(*featuresSet, index);
		}
	inputFeaturesSet = featuresSet;
	shot->featuresSetInput(inputFeaturesSet);

	PclNormalsCloudToPointCloudConverter normalsConverter;
	pclNormalsCloud = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	inputNormalsCloud = normalsConverter.Convert(pclNormalsCloud);
	shot->normalsCloudInput(inputNormalsCloud);

	outputWindowName = "Shot Descriptor 3D Result";
	}

ShotDescriptor3DTestInterface::~ShotDescriptor3DTestInterface()
	{
	delete(stubInputCache);
	delete(mockInputConverter);
	delete(stubOutputCache);
	delete(mockOutputConverter);
	delete(shot);
	delete(inputCloud);
	}

pcl::PointCloud<pcl::PointXYZ>::Ptr ShotDescriptor3DTestInterface::PreparePointCloudInput()
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr baseCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile("../../tests/Data/PointClouds/bunny0.ply", *baseCloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	unsigned selectionCounter = 0;
	unsigned const SELECTION_RATIO = 5;
	for(unsigned pointIndex = 0; pointIndex < baseCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = baseCloud->points.at(pointIndex);
		if (point.x == point.x && point.y == point.y && point.z == point.z)
			{
			if (selectionCounter == 0)
				{
				outputCloud->points.push_back(point);
				}
			selectionCounter = (selectionCounter+1)%SELECTION_RATIO;
			}
		}

	return outputCloud;
	}

void ShotDescriptor3DTestInterface::SetupMocksAndStubs()
	{
	stubInputCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>();
	mockInputConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	stubInputNormalsCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>();
	mockInputNormalsConverter = new Mocks::PointCloudToPclNormalsCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr, PointCloudToPclNormalsCloudConverter>::Instance(stubInputNormalsCache, mockInputNormalsConverter);

	stubOutputCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	mockOutputConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubOutputCache, mockOutputConverter);
	}

void ShotDescriptor3DTestInterface::SetupParameters()
	{
	AddParameter("GeneralParameters", "LocalReferenceFrameEstimationRadius", 0.10, 1.00, 0.01);
	AddParameter("GeneralParameters", "SearchRadius", 0.10, 1.00, 0.01);
	AddParameter("GeneralParameters", "OutputFormat", 0, 2);
	AddParameter("GeneralParameters", "EnableNormalsEstimation", 1, 1);
	AddParameter("GeneralParameters", "ForceNormalsEstimation", 1, 1);
	AddParameter("NormalEstimationParameters", "SearchRadius", 0.10, 1.00, 0.01);
	AddParameter("NormalEstimationParameters", "NeighboursSetSize", 0, 100);
	}

void ShotDescriptor3DTestInterface::DisplayResult()
	{
	VisualPointFeatureVector3DConstPtr featuresVector= shot->featuresSetWithDescriptorsOutput();

	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
	PRINT_TO_LOG("Number of feature points: ", GetNumberOfPoints(*featuresVector) );

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr featuresCloud = PrepareOutputCloud(featuresVector);

	pcl::visualization::PCLVisualizer viewer (outputWindowName);
    	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclCloudColor(pclCloud, 255, 255, 255);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbField(featuresCloud);
    	viewer.addPointCloud(pclCloud,pclCloudColor,"input");
	viewer.addPointCloud<pcl::PointXYZRGB> (featuresCloud, rgbField, "Keypoints");

    	while (!viewer.wasStopped ())
    		{
        	viewer.spinOnce();
        	pcl_sleep (0.01);
    		} 

	delete(featuresVector);
	}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ShotDescriptor3DTestInterface::PrepareOutputCloud(VisualPointFeatureVector3DConstPtr featuresVector)
	{
	const int featureIndexForRedColor = 2;
	const int featureIndexForGreenColor = 4;
	const int featureIndexForBlueColor = 5;

	float minR, maxR, minG, maxG, minB, maxB;
	GetComponentRange(featuresVector, featureIndexForRedColor, minR, maxR);
	GetComponentRange(featuresVector, featureIndexForGreenColor, minG, maxG);
	GetComponentRange(featuresVector, featureIndexForBlueColor, minB, maxB);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr featuresCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*featuresVector); pointIndex++)
		{
		pcl::PointXYZRGB newPoint;
		newPoint.x = GetXCoordinate(*featuresVector, pointIndex);
		newPoint.y = GetYCoordinate(*featuresVector, pointIndex);
		newPoint.z = GetZCoordinate(*featuresVector, pointIndex);
		newPoint.r = 250-150*( (GetDescriptorComponent(*featuresVector, pointIndex, featureIndexForRedColor) - minR)/(maxR - minR) );
		newPoint.g = 50+150*( (GetDescriptorComponent(*featuresVector, pointIndex, featureIndexForGreenColor) - minG)/(maxG - minG) );
		newPoint.b = 50+150*( (GetDescriptorComponent(*featuresVector, pointIndex, featureIndexForBlueColor) - minB)/(maxB - minB) );
		featuresCloud->points.push_back(newPoint);
		}

	return featuresCloud;
	}

void ShotDescriptor3DTestInterface::GetComponentRange(VisualPointFeatureVector3DConstPtr featuresVector, int componentIndex, float& min, float& max)
	{
	if ( GetNumberOfPoints(*featuresVector) == 0 )
		{
		return;
		}

	min = GetDescriptorComponent(*featuresVector, 0, componentIndex);
	max = min;
	for(int featureIndex = 1; featureIndex < GetNumberOfPoints(*featuresVector); featureIndex++)
		{
		ASSERT(GetNumberOfDescriptorComponents(*featuresVector, featureIndex) == SHOT_DESCRIPTOR_SIZE, "Shot descriptor size does not match size of received feature");
		if (max < GetDescriptorComponent(*featuresVector, featureIndex, componentIndex) )
			{
			max = GetDescriptorComponent(*featuresVector, featureIndex, componentIndex);
			}
		if (min > GetDescriptorComponent(*featuresVector, featureIndex, componentIndex) )
			{
			min = GetDescriptorComponent(*featuresVector, featureIndex, componentIndex);
			}
		}
	if (max == min)
		{
		max = min + 1;
		}	
	}


int main(int argc, char** argv)
	{
	ShotDescriptor3DTestInterface interface("HarrisDetector3D", 100, 40);
	interface.Run();
	};

/** @} */
