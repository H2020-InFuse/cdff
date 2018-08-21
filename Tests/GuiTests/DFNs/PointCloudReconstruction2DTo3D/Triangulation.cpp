/**
 * @author Alessandro Bianco
 */

/**
 * Test application for the DFN Triangulation
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <PointCloudReconstruction2DTo3D/Triangulation.hpp>
#include <CorrespondenceMap2D.hpp>
#include <Pose.hpp>
#include <PointCloud.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/DFNs/DFNTestInterface.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/core/core.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace CDFF::DFN::PointCloudReconstruction2DTo3D;
using namespace BaseTypesWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

class TriangulationTestInterface : public DFNTestInterface
{
	public:
		TriangulationTestInterface(std::string DFNName, int buttonWidth, int buttonHeight);
		~TriangulationTestInterface();

	private:
		Triangulation *triangulation;
		std::string outputWindowName;
		void SetupParameters();
		void DisplayResult();
	};

TriangulationTestInterface::TriangulationTestInterface(std::string DFNName, int buttonWidth, int buttonHeight)
	: DFNTestInterface(DFNName, buttonWidth, buttonHeight)
{
	triangulation = new Triangulation();
	SetDFN(triangulation);

	cv::Mat identityProjection(3, 4, CV_32FC1, cv::Scalar(0));
	identityProjection.at<float>(0,0) = 1;
	identityProjection.at<float>(1,1) = 1;
	identityProjection.at<float>(2,2) = 1;

	cv::Mat translationProjection(3, 4, CV_32FC1, cv::Scalar(0));
	translationProjection.at<float>(0,0) = 1;
	translationProjection.at<float>(1,1) = 1;
	translationProjection.at<float>(2,2) = 1;
	translationProjection.at<float>(0,3) = 1;

	cv::Mat cameraMatrix(3, 3, CV_32FC1, cv::Scalar(0));
	cameraMatrix.at<float>(0,0) = 1;
	cameraMatrix.at<float>(1,1) = 1;
	cameraMatrix.at<float>(0,2) = 0;
	cameraMatrix.at<float>(1,2) = 0;
	cameraMatrix.at<float>(2,2) = 1;

	cv::Mat sourceProjection = cameraMatrix * identityProjection;
	cv::Mat sinkProjection  = cameraMatrix * translationProjection;

	std::vector<Point3D> cube;
	for (unsigned x = 2; x <= 3; x++)
	{
		for (unsigned y = 2; y <= 3; y++)
		{
			for (unsigned z = 2; z <= 3; z++)
			{
				Point3D point;
				point.x = x;
				point.y = y;
				point.z = z;
				cube.push_back(point);
			}
		}
	}
	for (unsigned vertexIndex1 = 0; vertexIndex1 < 8; vertexIndex1++)
	{
		for (unsigned vertexIndex2 = vertexIndex1 + 1; vertexIndex2 < 8; vertexIndex2++)
		{
			for (float t = 0.1; t < 1.0; t += 0.1)
			{
				Point3D point;
				point.x = t*(cube.at(vertexIndex2).x - cube.at(vertexIndex1).x) + cube.at(vertexIndex1).x;
				point.y = t*(cube.at(vertexIndex2).y - cube.at(vertexIndex1).y) + cube.at(vertexIndex1).y;
				point.z = t*(cube.at(vertexIndex2).z - cube.at(vertexIndex1).z) + cube.at(vertexIndex1).z;
				cube.push_back(point);
			}
		}
	}

	CorrespondenceMap2D correspondenceMap;
	ClearCorrespondences(correspondenceMap);
	for (unsigned index = 0; index < cube.size(); index++)
	{
		cv::Mat point(4, 1, CV_32FC1);
		point.at<float>(0,0) = cube.at(index).x;
		point.at<float>(1,0) = cube.at(index).y;
		point.at<float>(2,0) = cube.at(index).z;
		point.at<float>(3,0) = 1;

		cv::Mat sourcePoint = sourceProjection * point;
		cv::Mat sinkPoint = sinkProjection * point;

		Point2D sink, source;
		source.x = sourcePoint.at<float>(0,0) / sourcePoint.at<float>(2,0);
		source.y = sourcePoint.at<float>(1,0) / sourcePoint.at<float>(2,0);
		sink.x = sinkPoint.at<float>(0,0) / sinkPoint.at<float>(2,0);
		sink.y = sinkPoint.at<float>(1,0) / sinkPoint.at<float>(2,0);
		AddCorrespondence(correspondenceMap, source, sink, 1);
	}
	triangulation->matchesInput(correspondenceMap);

	Pose3D pose;
	SetPosition(pose, 1, 0, 0);
	SetOrientation(pose, 0, 0, 0, 1);
	triangulation->poseInput(pose);

	outputWindowName = "Triangulation 2D to 3D Result";
}

TriangulationTestInterface::~TriangulationTestInterface()
{
	delete triangulation;
}

void TriangulationTestInterface::SetupParameters()
{
}

void TriangulationTestInterface::DisplayResult()
{
	const PointCloud& pointCloud = triangulation->pointcloudOutput();

	PRINT_TO_LOG("Processing time (seconds): ", GetLastProcessingTimeSeconds());
	PRINT_TO_LOG("Virtual memory used (kb): ", GetTotalVirtualMemoryUsedKB());
	PRINT_TO_LOG("Number of points: ", GetNumberOfPoints(pointCloud));

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for (int pointIndex = 0; pointIndex < GetNumberOfPoints(pointCloud); pointIndex++)
	{
		pcl::PointXYZ newPoint(
			GetXCoordinate(pointCloud, pointIndex),
			GetYCoordinate(pointCloud, pointIndex),
			GetZCoordinate(pointCloud, pointIndex)
		);
		pclPointCloud->points.push_back(newPoint);

		std::stringstream stream;
		stream << "Point " << pointIndex << ": " <<
			"(" << newPoint.x << ", " << newPoint.y << ", " << newPoint.z << ")";
		std::string string = stream.str();
		PRINT_TO_LOG("", string);
	}

	pcl::visualization::PCLVisualizer viewer(outputWindowName);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclCloudColor(pclPointCloud, 255, 255, 255);
	viewer.addPointCloud(pclPointCloud, pclCloudColor, "input");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		pcl_sleep(0.01);
	}
}

int main(int argc, char **argv)
{
	TriangulationTestInterface interface("Triangulation 2D to 3D", 100, 40);
	interface.Run();
};

/** @} */
