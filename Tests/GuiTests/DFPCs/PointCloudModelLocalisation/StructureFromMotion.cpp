/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StructureFromMotion.cpp
 * @date 26/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFPC StructureFromMotion.
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
#include <PointCloudModelLocalisation/StructureFromMotion.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <MatToFrameConverter.hpp>
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <VisualPointFeatureVector3D.hpp>
#include <Pose.hpp>
#include <PointCloud.hpp>

#include <Converters/SupportTypes.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToFrameConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector2DToMatConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToTransform3DConverter.hpp>
#include <Mocks/Common/Converters/Transform3DToMatConverter.hpp>
#include <Mocks/Common/Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclNormalsCloudConverter.hpp>
#include <Mocks/Common/Converters/EigenTransformToTransform3DConverter.cpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>

#include <Stubs/DFPCs/PointCloudModelLocalisation/ObservedScene.hpp>

using namespace dfpc_ci;
using namespace FrameWrapper;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;
using namespace Common;
using namespace PointCloudWrapper;
using namespace Converters::SupportTypes;

int main(int argc, char** argv)
	{
	Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubFrameCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	Mocks::FrameToMatConverter* mockFrameConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubFrameCache, mockFrameConverter);

	Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubInverseFrameCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	Mocks::MatToFrameConverter* mockInverseFrameConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubInverseFrameCache, mockInverseFrameConverter);

	Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>* stubMatToVectorCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>();
	Mocks::MatToVisualPointFeatureVector2DConverter* mockMatToVectorConverter = new Mocks::MatToVisualPointFeatureVector2DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector2DConstPtr, MatToVisualPointFeatureVector2DConverter>::Instance(stubMatToVectorCache, mockMatToVectorConverter);

	Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>* stubVectorToMatCache = new Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>();
	Mocks::VisualPointFeatureVector2DToMatConverter* mockVectorToMatConverter = new Mocks::VisualPointFeatureVector2DToMatConverter();
	ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Instance(stubVectorToMatCache, mockVectorToMatConverter);

	Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* stubEssentialPoseCache = new Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>();
	Mocks::MatToPose3DConverter* mockEssentialPoseConverter = new Mocks::MatToPose3DConverter();
	ConversionCache<cv::Mat, Pose3DConstPtr, MatToPose3DConverter>::Instance(stubEssentialPoseCache, mockEssentialPoseConverter);

	Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>* stubTriangulationPoseCache = new Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>();
	Mocks::Pose3DToMatConverter* mockTriangulationPoseConverter = new Mocks::Pose3DToMatConverter();
	ConversionCache<Pose3DConstPtr, cv::Mat, Pose3DToMatConverter>::Instance(stubTriangulationPoseCache, mockTriangulationPoseConverter);

	Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>* stubCloudCache = new Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>;
	Mocks::PclPointCloudToPointCloudConverter* mockCloudConverter = new Mocks::PclPointCloudToPointCloudConverter();
	ConversionCache<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr, PclPointCloudToPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);

	Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>* stubInverseCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>;
	Mocks::PointCloudToPclPointCloudConverter* mockInverseCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInverseCloudCache, mockInverseCloudConverter);

	Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>* stubFeatures3dCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	Mocks::MatToVisualPointFeatureVector3DConverter* mockFeatures3dConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubFeatures3dCache, mockFeatures3dConverter);

	Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>* stubInputNormalsCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>();
	Mocks::PointCloudToPclNormalsCloudConverter* mockInputNormalsConverter = new Mocks::PointCloudToPclNormalsCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr, PointCloudToPclNormalsCloudConverter>::Instance(stubInputNormalsCache, mockInputNormalsConverter);

	Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>* stubInputCache = new Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>();
	Mocks::VisualPointFeatureVector3DToPclPointCloudConverter* mockInputConverter = new Mocks::VisualPointFeatureVector3DToPclPointCloudConverter();
	ConversionCache<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>* stubOutputCache = new Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>();
	Mocks::EigenTransformToTransform3DConverter* mockOutputConverter = new Mocks::EigenTransformToTransform3DConverter();
	ConversionCache<Eigen::Matrix4f, Transform3DConstPtr, EigenTransformToTransform3DConverter>::Instance(stubOutputCache, mockOutputConverter);

	WRITE_TO_LOG("Configure", "");
	Map* map = new ObservedScene();
	StructureFromMotion structureFromMotion(map);
	structureFromMotion.setConfigurationFile("../../tests/ConfigurationFiles/DFPCs/PointCloudModelLocalisation/DfpcStructureFromMotion_conf01.yaml");
	structureFromMotion.configure();
	WRITE_TO_LOG("Configure", "Completed");

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclModelCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile("../../tests/Data/PointClouds/bunny0.ply", *pclCloud);
	const unsigned SELECTION_RATIO = 10;
	unsigned selectionCounter = 0;
	for(unsigned pointIndex = 0; pointIndex < pclCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = pclCloud->points.at(pointIndex);
		if (point.x == point.x && point.y == point.y && point.z == point.z)
			{
			if (selectionCounter == 0)
				{
				pclModelCloud->points.push_back(point);
				}
			selectionCounter = (selectionCounter+1)%SELECTION_RATIO;
			}
		}
	PclPointCloudToPointCloudConverter pclConverter;
	PointCloudConstPtr modelCloud = pclConverter.Convert(pclModelCloud);
	structureFromMotion.modelInput(modelCloud);
	WRITE_TO_LOG("Model Loaded", "");

	cv::Mat doubleImageA = cv::imread("../../tests/Data/Images/SmestechLabA.jpg", cv::IMREAD_COLOR);
	cv::Mat doubleImageB = cv::imread("../../tests/Data/Images/SmestechLabB.jpg", cv::IMREAD_COLOR);
	cv::Mat doubleImageC = cv::imread("../../tests/Data/Images/SmestechLabC.jpg", cv::IMREAD_COLOR);
	cv::Mat doubleImageD = cv::imread("../../tests/Data/Images/SmestechLabD.jpg", cv::IMREAD_COLOR);
	cv::Mat doubleImageE = cv::imread("../../tests/Data/Images/SmestechLabE.jpg", cv::IMREAD_COLOR);
	cv::Mat doubleImageF = cv::imread("../../tests/Data/Images/SmestechLabF.jpg", cv::IMREAD_COLOR);
	cv::Mat doubleImageG = cv::imread("../../tests/Data/Images/SmestechLabG.jpg", cv::IMREAD_COLOR);
	
	cv::Mat firstCvImage = doubleImageA( cv::Rect(0,0,doubleImageA.cols/2,doubleImageA.rows) );
	cv::Mat secondCvImage = doubleImageB( cv::Rect(0,0,doubleImageB.cols/2,doubleImageB.rows) );
	cv::Mat thirdCvImage = doubleImageC( cv::Rect(0,0,doubleImageC.cols/2,doubleImageC.rows) );
	cv::Mat fourthCvImage = doubleImageD( cv::Rect(0,0,doubleImageD.cols/2,doubleImageD.rows) );
	cv::Mat fifthCvImage = doubleImageE( cv::Rect(0,0,doubleImageE.cols/2,doubleImageE.rows) );
	cv::Mat sixthCvImage = doubleImageF( cv::Rect(0,0,doubleImageF.cols/2,doubleImageF.rows) );
	cv::Mat seventhCvImage = doubleImageG( cv::Rect(0,0,doubleImageG.cols/2,doubleImageG.rows) );

	MatToFrameConverter converter;
	FrameConstPtr firstImage = converter.Convert(firstCvImage);
	FrameConstPtr secondImage = converter.Convert(secondCvImage);
	FrameConstPtr thirdImage = converter.Convert(thirdCvImage);
	FrameConstPtr fourthImage = converter.Convert(fourthCvImage);
	FrameConstPtr fifthImage = converter.Convert(fifthCvImage);
	FrameConstPtr sixthImage = converter.Convert(sixthCvImage);
	FrameConstPtr seventhImage = converter.Convert(seventhCvImage);
	
	WRITE_TO_LOG("First Image Input", "");
	structureFromMotion.imageInput(firstImage);
	structureFromMotion.process();
	WRITE_TO_LOG("Result", structureFromMotion.successOutput() );

	WRITE_TO_LOG("Second Image Input", "");
	structureFromMotion.imageInput(secondImage);
	structureFromMotion.process();
	WRITE_TO_LOG("Result", structureFromMotion.successOutput() );

	WRITE_TO_LOG("Third Image Input", "");
	structureFromMotion.imageInput(thirdImage);
	structureFromMotion.process();
	WRITE_TO_LOG("Result", structureFromMotion.successOutput() );

	WRITE_TO_LOG("Fourth Image Input", "");
	structureFromMotion.imageInput(fourthImage);
	structureFromMotion.process();
	WRITE_TO_LOG("Result", structureFromMotion.successOutput() );

	WRITE_TO_LOG("Fifth Image Input", "");
	structureFromMotion.imageInput(fifthImage);
	structureFromMotion.process();
	WRITE_TO_LOG("Result", structureFromMotion.successOutput() );

	WRITE_TO_LOG("Sixth Image Input", "");
	structureFromMotion.imageInput(sixthImage);
	structureFromMotion.process();
	WRITE_TO_LOG("Result", structureFromMotion.successOutput() );

	WRITE_TO_LOG("Seventh Image Input", "");
	structureFromMotion.imageInput(seventhImage);
	structureFromMotion.process();
	WRITE_TO_LOG("Result", structureFromMotion.successOutput() );

	PRINT_LOG();
	PointCloudConstPtr pointCloud = structureFromMotion.pointCloudOutput();
	if (pointCloud == NULL)
		{
		return 0;
		}


	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*pointCloud); pointIndex++)
		{
		pcl::PointXYZ newPoint(GetXCoordinate(*pointCloud, pointIndex), GetYCoordinate(*pointCloud, pointIndex), GetZCoordinate(*pointCloud, pointIndex) );
		std::stringstream stream;
		pclPointCloud->points.push_back(newPoint);

		stream << "Point "<<pointIndex<<": ("<<newPoint.x<<", "<<newPoint.y<<", "<<newPoint.z<<")";
		std::string string = stream.str();
		PRINT_TO_LOG("", string );
		}

	pcl::visualization::PCLVisualizer viewer ("Output Cloud");
    	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclCloudColor(pclPointCloud, 255, 255, 255);
    	viewer.addPointCloud(pclPointCloud,pclCloudColor,"input");

    	while (!viewer.wasStopped ())
    		{
        	viewer.spinOnce();
        	pcl_sleep (0.01);
    		} 
	return 0;
	};

/** @} */
