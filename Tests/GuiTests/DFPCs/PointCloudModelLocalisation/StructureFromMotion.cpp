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
#include<pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <MatToFrameConverter.hpp>
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <Pose.hpp>
#include <PointCloud.hpp>

#include <ConversionCache/ConversionCache.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToFrameConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector2DToMatConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToTransform3DConverter.hpp>
#include <Mocks/Common/Converters/Transform3DToMatConverter.hpp>

using namespace dfpc_ci;
using namespace FrameWrapper;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace PoseWrapper;
using namespace Common;
using namespace PointCloudWrapper;

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

	PRINT_TO_LOG("Configure", "");
	StructureFromMotion structureFromMotion;
	structureFromMotion.setConfigurationFile("../../tests/ConfigurationFiles/DFPCs/PointCloudModelLocalisation/DfpcStructureFromMotion_conf01.yaml");
	structureFromMotion.configure();
	PRINT_TO_LOG("Configure", "Completed");

	cv::Mat doubleImageA = cv::imread("../../tests/Data/Images/SmestechLabA.jpg", cv::IMREAD_COLOR);
	cv::Mat doubleImageB = cv::imread("../../tests/Data/Images/SmestechLabB.jpg", cv::IMREAD_COLOR);
	cv::Mat doubleImageC = cv::imread("../../tests/Data/Images/SmestechLabC.jpg", cv::IMREAD_COLOR);
	
	cv::Mat firstCvImage = doubleImageA( cv::Rect(0,0,doubleImageA.cols/2,doubleImageA.rows) );
	cv::Mat secondCvImage = doubleImageB( cv::Rect(0,0,doubleImageB.cols/2,doubleImageB.rows) );
	cv::Mat thirdCvImage = doubleImageC( cv::Rect(0,0,doubleImageC.cols/2,doubleImageC.rows) );

	MatToFrameConverter converter;
	FrameConstPtr firstImage = converter.Convert(firstCvImage);
	FrameConstPtr secondImage = converter.Convert(secondCvImage);
	FrameConstPtr thirdImage = converter.Convert(thirdCvImage);
	
	PRINT_TO_LOG("First Image Input", "");
	structureFromMotion.imageInput(firstImage);
	structureFromMotion.process();
	PRINT_TO_LOG("Result", structureFromMotion.successOutput() );

	PRINT_TO_LOG("Second Image Input", "");
	structureFromMotion.imageInput(secondImage);
	structureFromMotion.process();
	PRINT_TO_LOG("Result", structureFromMotion.successOutput() );

	PRINT_TO_LOG("Third Image Input", "");
	structureFromMotion.imageInput(thirdImage);
	structureFromMotion.process();
	PRINT_TO_LOG("Result", structureFromMotion.successOutput() );

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
