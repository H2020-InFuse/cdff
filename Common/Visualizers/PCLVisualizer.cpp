/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup Visualizers
 * @{
 */

#include "PCLVisualizer.hpp"
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Errors/Assert.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;
using namespace Converters;

#define RETURN_IF_DISABLED \
	if (!enabled) \
	{ \
		return; \
	}

#define RETURN_IF_SAVING_DISABLED \
	if (!enabledSaving) \
	{ \
		return; \
	}

namespace Visualizers
{

void PclVisualizer::ShowPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud)
{
	RETURN_IF_DISABLED

	std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > pointCloudsList;
	pointCloudsList.push_back(pointCloud);

	ShowPointClouds(pointCloudsList);
}

void PclVisualizer::ShowPointClouds(std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > pointCloudsList)
{
	RETURN_IF_DISABLED

	pcl::visualization::PCLVisualizer viewer(WINDOW_NAME);
	ASSERT(pointCloudsList.size() <= MAX_POINT_CLOUDS, "PclVisualizer, too many point clouds to display");
	viewer.addCoordinateSystem(0.1);

	for (unsigned pointCloudIndex = 0; pointCloudIndex < pointCloudsList.size(); pointCloudIndex++)
	{
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud = pointCloudsList.at(pointCloudIndex);
		Color cloudColor = COLORS_LIST[pointCloudIndex];
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclCloudColor(pointCloud, cloudColor.r, cloudColor.g, cloudColor.b);

		std::stringstream cloudId;
		cloudId << "Point Cloud "<< pointCloudIndex;
		viewer.addPointCloud(pointCloud, pclCloudColor, cloudId.str());
	}

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		pcl_sleep(0.01);
	}

	viewer.removeAllPointClouds();
	viewer.close();
}

void PclVisualizer::ShowPointCloud(PointCloudConstPtr pointCloud)
{
	RETURN_IF_DISABLED

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclPointCloud = PointCloudToPclPointCloudConverter().Convert(pointCloud);

	ShowPointCloud(pclPointCloud);
}

void PclVisualizer::ShowVisualFeatures(PointCloudConstPtr pointCloud, VisualPointFeatureVector3DConstPtr featuresVector)
{
	static const int IndependentObjectLimit = 50;

	RETURN_IF_DISABLED

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclPointCloud = PointCloudToPclPointCloudConverter().Convert(pointCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr featuresCloud(new pcl::PointCloud<pcl::PointXYZ>());

	for (int pointIndex = 0; pointIndex < GetNumberOfPoints(*featuresVector); pointIndex++)
	{
		VisualPointType type = GetPointType(*featuresVector, pointIndex);
		pcl::PointXYZ point;
		if (type == VISUAL_POINT_POSITION)
		{
			point.x = GetXCoordinate(*featuresVector, pointIndex);
			point.y = GetYCoordinate(*featuresVector, pointIndex);
			point.z = GetZCoordinate(*featuresVector, pointIndex);
		}
		else if (type == VISUAL_POINT_REFERENCE)
		{
			point = pclPointCloud->points.at(GetReferenceIndex(*featuresVector, pointIndex));
		}
		else
		{
			ASSERT(false, "PclVisualizer, unhandled point type in VisualPointFeatureVector3D");
		}
		featuresCloud->points.push_back(point);
	}

	if (featuresCloud->points.size() < IndependentObjectLimit)
		{
		ShowPointCloudsAndSpheres(pclPointCloud, featuresCloud);
		}
	else
		{
		std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > pointsCloudList;
		pointsCloudList.push_back(pclPointCloud);
		pointsCloudList.push_back(featuresCloud);

		ShowPointClouds(pointsCloudList);
		}
}

void PclVisualizer::ShowImage(pcl::PointCloud<pcl::RGB>::ConstPtr image)
{
	RETURN_IF_DISABLED

	pcl::visualization::PCLVisualizer viewer(WINDOW_NAME);
	viewer.addCoordinateSystem(0.1);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr imageCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	for (unsigned row = 0; row < image->height; row++)
	{
		for (unsigned column = 0; column < image->width; column++)
		{
			pcl::RGB point = image->points.at(row * image->width + column);
			pcl::PointXYZRGB imageCloudPoint;
			imageCloudPoint.x = static_cast<float>(column) * 0.01;
			imageCloudPoint.y = - static_cast<float>(row) * 0.01;
			imageCloudPoint.z = 0;
			imageCloudPoint.r = point.r;
			imageCloudPoint.g = point.g;
			imageCloudPoint.b = point.b;
			imageCloud->points.push_back(imageCloudPoint);
		}
	}

	viewer.addPointCloud<pcl::PointXYZRGB>(imageCloud);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		pcl_sleep(0.01);
	}

	viewer.removeAllPointClouds();
	viewer.close();
}

void PclVisualizer::ShowPoses(std::vector<PoseWrapper::Pose3D> poseList)
{
	RETURN_IF_DISABLED

	pcl::visualization::PCLVisualizer viewer(WINDOW_NAME);
	viewer.addCoordinateSystem(0.1);

	int index = 0;
	for (std::vector<PoseWrapper::Pose3D>::iterator pose = poseList.begin(); pose != poseList.end(); pose++)
	{
		double x = GetXPosition(*pose);
		double y = GetYPosition(*pose);
		double z = GetZPosition(*pose);
		double qx = GetXOrientation(*pose);
		double qy = GetYOrientation(*pose);
		double qz = GetZOrientation(*pose);
		double qw = GetWOrientation(*pose);

		pcl::PointXYZ point(x, y, z);
		pcl::PointXYZ axisX, axisY, axisZ;

		axisZ.x = 0.4 * (2*qx*qz + 2*qy*qw) + x;
		axisZ.y = 0.4 * (2*qy*qz - 2*qx*qw) + y;
		axisZ.z = 0.4 * (-qx*qx - qy*qy + qz*qz + qw*qw) + z;

		axisX.x = 0.1 * (qx*qx - qy*qy - qz*qz + qw*qw) + x;
		axisX.y = 0.1 * (2*qx*qy + 2*qz*qw) + y;
		axisX.z = 0.1 * (2*qx*qz - 2*qy*qw) + z;

		axisY.x = 0.2 * (2*qx*qy - 2*qz*qw) + x;
		axisY.y = 0.2 * (-qx*qx + qy*qy - qz*qz + qw*qw) + y;
		axisY.z = 0.2 * (2*qx*qw + 2*qy*qz) + z;

		viewer.addLine<pcl::PointXYZ>(point, axisX, "lineX"+std::to_string(index));
		viewer.addLine<pcl::PointXYZ>(point, axisY, "lineY"+std::to_string(index));
		viewer.addLine<pcl::PointXYZ>(point, axisZ, "lineZ"+std::to_string(index));
		index++;
	}

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		pcl_sleep(0.01);
	}

	viewer.removeAllPointClouds();
	viewer.close();
}

void PclVisualizer::PlacePointCloud(PointCloudConstPtr sceneCloud, PointCloudConstPtr objectCloud, Pose3DConstPtr objectPoseInScene)
{
	RETURN_IF_DISABLED

	PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclSceneCloud = pointCloudToPclPointCloud.Convert(sceneCloud);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclObjectCloud = pointCloudToPclPointCloud.Convert(objectCloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedObject(new pcl::PointCloud<pcl::PointXYZ>());

	for (unsigned pointIndex = 0; pointIndex < pclObjectCloud->points.size(); pointIndex++)
	{
		pcl::PointXYZ transformedPoint = TransformPoint(pclObjectCloud->points.at(pointIndex), objectPoseInScene);
		transformedObject->points.push_back(transformedPoint);
	}

	std::vector< pcl::PointCloud<pcl::PointXYZ>::ConstPtr > pointsCloudList;
	pointsCloudList.push_back(pclSceneCloud);
	pointsCloudList.push_back(transformedObject);

	ShowPointClouds(pointsCloudList);
}

void PclVisualizer::ShowMatches(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud2, const std::vector<int>& indexList1, const std::vector<int>& indexList2)
{
	RETURN_IF_DISABLED

	pcl::visualization::PCLVisualizer viewer(WINDOW_NAME);
	ASSERT(indexList1.size() == indexList2.size(), "PclVisualizer, indexList1 has different size with indexList2");
	viewer.addCoordinateSystem(0.1);

	Color cloudColor1 = COLORS_LIST[0];
	Color cloudColor2 = COLORS_LIST[1];

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclCloudColor1(pointCloud1, cloudColor1.r, cloudColor1.g, cloudColor1.b);
	viewer.addPointCloud(pointCloud1, pclCloudColor1, "cloud1");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclCloudColor2(pointCloud2, cloudColor2.r, cloudColor2.g, cloudColor2.b);
	viewer.addPointCloud(pointCloud2, pclCloudColor2, "cloud2");

	for (int indexIndex = 0; indexIndex < indexList1.size(); indexIndex++)
	{
		int index1 = indexList1.at(indexIndex);
		int index2 = indexList2.at(indexIndex);
		pcl::PointXYZ point1 = pointCloud1->points.at(index1);
		pcl::PointXYZ point2 = pointCloud2->points.at(index2);

		viewer.addLine<pcl::PointXYZ>(point1, point2, "line"+std::to_string(indexIndex));
	}

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		pcl_sleep(0.01);
	}

	viewer.removeAllPointClouds();
	viewer.close();
}

void PclVisualizer::SavePointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, unsigned period)
{
	static unsigned time = 1;
	static unsigned saveTime = 0;

	RETURN_IF_SAVING_DISABLED

	if (time % period != 0)
	{
		time++;
		return;
	}

	std::stringstream cloudOutputPath;
	cloudOutputPath << SAVE_FILE_BASE_NAME << saveTime << SAVE_FILE_EXTENSION;

	pcl::PLYWriter writer;
	writer.write(cloudOutputPath.str(), *pointCloud);

	saveTime++;
	time = 1;
}

void PclVisualizer::SavePointCloud(PointCloudConstPtr pointCloud, unsigned period)
{
	RETURN_IF_SAVING_DISABLED

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclPointCloud = PointCloudToPclPointCloudConverter().Convert(pointCloud);

	SavePointCloud(pclPointCloud, period);
}

void PclVisualizer::Enable()
{
	enabled = true;
}

void PclVisualizer::Disable()
{
	enabled = false;
}

void PclVisualizer::EnableSaving()
{
	enabledSaving = true;
}

void PclVisualizer::DisableSaving()
{
	enabledSaving = false;
}

PclVisualizer::PclVisualizer()
{
}

const std::string PclVisualizer::WINDOW_NAME = "CDFF PCL-Based Visualizer";
const std::string PclVisualizer::SAVE_FILE_BASE_NAME = "VisualizerCloud";
const std::string PclVisualizer::SAVE_FILE_EXTENSION = ".ply";
const PclVisualizer::Color PclVisualizer::COLORS_LIST[MAX_POINT_CLOUDS] =
{
	{.r = 255, .g = 255, .b = 255},
	{.r = 200, .g = 0, .b = 0},
	{.r = 0, .g = 200, .b = 0},
	{.r = 0, .g = 0, .b = 200},
	{.r = 200, .g = 200, .b = 0},
	{.r = 200, .g = 0, .b = 200},
	{.r = 0, .g = 200, .b = 200},
	{.r = 200, .g = 100, .b = 100},
	{.r = 100, .g = 200, .b = 100},
	{.r = 100, .g = 100, .b = 200}
};
bool PclVisualizer::enabled = false;
bool PclVisualizer::enabledSaving = false;

pcl::PointXYZ PclVisualizer::TransformPoint(pcl::PointXYZ point, Transform3DConstPtr transform)
{
	Eigen::Quaternion<float> rotation(GetWRotation(*transform), GetXRotation(*transform), GetYRotation(*transform), GetZRotation(*transform));
	Eigen::Translation<float, 3> translation( GetXPosition(*transform), GetYPosition(*transform), GetZPosition(*transform));
	Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> affineTransform = translation * rotation;

	Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
	Eigen::Vector3f eigenTransformedPoint = affineTransform * eigenPoint;
	pcl::PointXYZ transformedPoint;
	transformedPoint.x = eigenTransformedPoint.x();
	transformedPoint.y = eigenTransformedPoint.y();
	transformedPoint.z = eigenTransformedPoint.z();
	return transformedPoint;
}

void PclVisualizer::ShowPointCloudsAndSpheres(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sphereCloud)
	{
	static const float radius = 0.003;
	RETURN_IF_DISABLED

	pcl::visualization::PCLVisualizer viewer(WINDOW_NAME);
	viewer.addCoordinateSystem(0.1);

	Color cloudColor = COLORS_LIST[0];
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclCloudColor(pointCloud, cloudColor.r, cloudColor.g, cloudColor.b);
	viewer.addPointCloud(pointCloud, pclCloudColor, "main cloud");

	int numberOfPoints = sphereCloud->points.size();
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		std::string sphereId = "sphere_" + std::to_string(pointIndex);
		viewer.addSphere( sphereCloud->points.at(pointIndex), radius, COLORS_LIST[1].r, COLORS_LIST[1].g, COLORS_LIST[1].b, sphereId);
		}

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
		pcl_sleep(0.01);
	}

	viewer.removeAllPointClouds();
	viewer.close();
	}

}

/** @} */
