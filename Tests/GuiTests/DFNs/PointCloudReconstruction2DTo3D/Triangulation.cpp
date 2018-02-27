/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Triangulation.cpp
 * @date 29/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN Triangulation.
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
#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/Transform3DToMatConverter.hpp>
#include <Errors/Assert.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include<pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace dfn_ci;
using namespace Converters;
using namespace Common;
using namespace PoseWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PointCloudWrapper;
using namespace MatrixWrapper;
using namespace BaseTypesWrapper;

class TriangulationTestInterface : public DFNTestInterface
	{
	public:
		TriangulationTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~TriangulationTestInterface();
	protected:

	private:
		Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>* stubTriangulationPoseCache;
		Mocks::Pose3DToMatConverter* mockTriangulationPoseConverter;

		Triangulation* triangulation;

		std::string outputWindowName;

		void SetupMocksAndStubs();
		void SetupParameters();
		void DisplayResult();
	};

TriangulationTestInterface::TriangulationTestInterface(std::string dfnName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(dfnName, buttonWidth, buttonHeight)
	{
	triangulation = new Triangulation();
	SetDFN(triangulation);

	cv::Mat identityProjection(3, 4, CV_32FC1, cv::Scalar(0) );
	identityProjection.at<float>(0,0) = 1;
	identityProjection.at<float>(1,1) = 1;
	identityProjection.at<float>(2,2) = 1;

	cv::Mat translationProjection(3, 4, CV_32FC1, cv::Scalar(0) );
	translationProjection.at<float>(0,0) = 1;
	translationProjection.at<float>(1,1) = 1;
	translationProjection.at<float>(2,2) = 1;
	translationProjection.at<float>(0,3) = 1;

	cv::Mat cameraMatrix(3, 3, CV_32FC1, cv::Scalar(0) );
	cameraMatrix.at<float>(0,0) = 1;
	cameraMatrix.at<float>(1,1) = 1;
	cameraMatrix.at<float>(0,2) = 0;
	cameraMatrix.at<float>(1,2) = 0;
	cameraMatrix.at<float>(2,2) = 1;

	cv::Mat sourceProjection = cameraMatrix * translationProjection;
	cv::Mat sinkProjection = cameraMatrix * identityProjection;

	std::vector<BaseTypesWrapper::Point3D> cube;
	for(unsigned x = 2; x<=3; x++)
		{
		for(unsigned y=2; y<=3; y++)
			{
			for(unsigned z=2; z<=3; z++)
				{
				BaseTypesWrapper::Point3D point;
				point.x = x;
				point.y = y;
				point.z = z;
				cube.push_back(point);
				}
			}
		}

	for(unsigned vertexIndex1 = 0; vertexIndex1 < 8; vertexIndex1++)
		{
		for(unsigned vertexIndex2 = vertexIndex1 +1; vertexIndex2 < 8; vertexIndex2++)
			{
			for(float t = 0.1; t<1.0; t+=0.1)
				{
				BaseTypesWrapper::Point3D point;
				point.x = t*(cube.at(vertexIndex2).x - cube.at(vertexIndex1).x) + cube.at(vertexIndex1).x;	
				point.y = t*(cube.at(vertexIndex2).y - cube.at(vertexIndex1).y) + cube.at(vertexIndex1).y;			
				point.z = t*(cube.at(vertexIndex2).z - cube.at(vertexIndex1).z) + cube.at(vertexIndex1).z;
				cube.push_back(point);					
				}
			}
		}
		

	CorrespondenceMap2DPtr correspondenceMap = new CorrespondenceMap2D;
	ClearCorrespondences(*correspondenceMap);
	for(unsigned index = 0; index < cube.size(); index++)
		{
		cv::Mat point(4, 1, CV_32FC1);
		point.at<float>(0,0) = cube.at(index).x;
		point.at<float>(1,0) = cube.at(index).y;
		point.at<float>(2,0) = cube.at(index).z;
		point.at<float>(3,0) = 1;

		cv::Mat sourcePoint = sourceProjection * point;
		cv::Mat sinkPoint = sinkProjection * point;

		BaseTypesWrapper::Point2D sink, source;
		source.x = sourcePoint.at<float>(0,0) / sourcePoint.at<float>(2,0);
		source.y = sourcePoint.at<float>(1,0) / sourcePoint.at<float>(2,0);
		sink.x = sinkPoint.at<float>(0,0) / sinkPoint.at<float>(2,0);
		sink.y = sinkPoint.at<float>(1,0) / sinkPoint.at<float>(2,0);
		AddCorrespondence(*correspondenceMap, source, sink, 1);
		}
	triangulation->correspondenceMapInput(correspondenceMap);

	Pose3DPtr pose = new Pose3D();
	SetPosition(*pose, 1, 0, 0);
	SetOrientation(*pose, 0, 0, 0, 1);
	triangulation->poseInput(pose);

	outputWindowName = "Triangulation 2D to 3D Result";
	}

TriangulationTestInterface::~TriangulationTestInterface()
	{
	delete(triangulation);
	}

void TriangulationTestInterface::SetupMocksAndStubs()
	{
	stubTriangulationPoseCache = new Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>();
	mockTriangulationPoseConverter = new Mocks::Pose3DToMatConverter();
	ConversionCache<Pose3DConstPtr, cv::Mat, Pose3DToMatConverter>::Instance(stubTriangulationPoseCache, mockTriangulationPoseConverter);
	}

void TriangulationTestInterface::SetupParameters()
	{

	}

void TriangulationTestInterface::DisplayResult()
	{
	PointCloudWrapper::PointCloudConstPtr pointCloud= triangulation->pointCloudOutput();

	PRINT_TO_LOG("The processing took (seconds): ", GetLastProcessingTimeSeconds() );
	PRINT_TO_LOG("Virtual Memory used (Kb): ", GetTotalVirtualMemoryUsedKB() );
	PRINT_TO_LOG("Number of points: ", GetNumberOfPoints(*pointCloud) );

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

	pcl::visualization::PCLVisualizer viewer (outputWindowName);
    	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclCloudColor(pclPointCloud, 255, 255, 255);
    	viewer.addPointCloud(pclPointCloud,pclCloudColor,"input");

    	while (!viewer.wasStopped ())
    		{
        	viewer.spinOnce();
        	pcl_sleep (0.01);
    		} 

	delete(pointCloud);
	}


int main(int argc, char** argv)
	{
	TriangulationTestInterface interface("Triangulation 2D to 3D", 100, 40);
	interface.Run();
	};

/** @} */
