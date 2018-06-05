/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OutliersDetector.cpp
 * @date 14/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the OutliersDetector class.
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
#include "OutliersDetector.hpp"
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <Errors/Assert.hpp>
#include <boost/make_shared.hpp>

#define DELETE_IF_NOT_NULL(pointer) \
	if (pointer != NULL) \
		{ \
		delete(pointer); \
		}
	

namespace DataGenerators
{

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
OutliersDetector::OutliersDetector(std::string inputCloudFilePath, std::string outliersFilePath) :
	originalCloud(new pcl::PointCloud<pcl::PointXYZ>),
	outliersCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
	visualizer(new pcl::visualization::PCLVisualizer("Outliers Detector"))
	{
	this->inputCloudFilePath = inputCloudFilePath;
	this->outliersFilePath = outliersFilePath;

	LoadCloud();
	LoadOutliers();

	visualizer->registerKeyboardCallback(OutliersDetector::KeyboardButtonCallback, this);
	visualizer->registerPointPickingCallback(OutliersDetector::PointPickingCallback, this);

	pointCloudColorHandler = boost::make_shared<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> >(outliersCloud);
	visualizer->addPointCloud< pcl::PointXYZRGB >(outliersCloud, *pointCloudColorHandler, "outliersCloud");

	detectorIsActive = true;
	cloudChangedSinceLastVisualization = true;
	focusChangedSinceLastVisualization = true;
	outliersBoxActive = false;
	}

OutliersDetector::~OutliersDetector()
	{	
	visualizer->close();
	}

void OutliersDetector::Run()
	{
	while(detectorIsActive && !visualizer->wasStopped())
		{
		if (cloudChangedSinceLastVisualization)
			{
			std::lock_guard<std::mutex> guard(inputMutex);
			if (focusChangedSinceLastVisualization)
				{
				PrepareCloudToVisualize();
				}
			DrawOutliers();
			cloudChangedSinceLastVisualization = false;
			}

		visualizer->spinOnce(100);
		}
	}


/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const std::vector<cv::Scalar> OutliersDetector::COLORS_LIST =
	{
	cv::Scalar(0, 0, 255),
	cv::Scalar(0, 255, 0),
	cv::Scalar(255, 0, 0),
	cv::Scalar(0, 255, 255),
	cv::Scalar(255, 255, 0),
	cv::Scalar(255, 0, 255),
	cv::Scalar(128, 128, 255),
	cv::Scalar(128, 255, 128),
	cv::Scalar(255, 128, 128),
	cv::Scalar(128, 255, 255),
	cv::Scalar(255, 255, 128),
	cv::Scalar(255, 128, 255),
	cv::Scalar(0, 0, 128),
	cv::Scalar(0, 128, 0),
	cv::Scalar(128, 0, 0),
	cv::Scalar(0, 128, 128),
	cv::Scalar(128, 128, 0),
	cv::Scalar(128, 0, 128)
	};


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
#define PRINT_CLOUD_LOCATION_INFO(location, center, radius) \
	{ \
	std::stringstream locationStream; \
	locationStream << location << "   Center: (" << center.x << ", " << center.y << ", " << center.z << ")  3D Radius: " << radius; \
	PRINT_TO_LOG(locationStream.str(), ""); \
	}

void OutliersDetector::LoadCloud()
	{
	pcl::io::loadPLYFile(inputCloudFilePath, *originalCloud);
	originalCloudSearchTree.setInputCloud(originalCloud);
	PRINT_TO_LOG("Number of points in source cloud: ", (originalCloud->points.size()) );
	ComputeCloudCenter(originalCloud, visualizationCenter);
	visualizationRadius = ComputeCloudRadius(originalCloud, visualizationCenter) / 10;

	visualizationCircleSize = ComputeMinimumDistanceBetweenPoints(originalCloud);
	PRINT_TO_LOG("Minimum keypoint distance:", visualizationCircleSize);

	PRINT_CLOUD_LOCATION_INFO("Center-Radius:", visualizationCenter, visualizationRadius);
	}

void OutliersDetector::PointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* userdata)
	{
	((OutliersDetector*)userdata)->PointPickingCallback(event);
	}

void OutliersDetector::PointPickingCallback(const pcl::visualization::PointPickingEvent& event)
	{
	std::lock_guard<std::mutex> guard(inputMutex);

	int32_t pointIndex = event.getPointIndex();
	if(pointIndex == -1)
		{
		return;
		}

	int32_t originalPointIndex = visualizedIndicesList.at(pointIndex);
	float x, y, z;
	event.getPoint(x,y,z);

	//Looking for the point index by mean of coordinates
	bool found = false;
	for(int pointIndexr = 0; pointIndexr < outliersCloud->points.size() && !found; pointIndexr++)
		{
		pcl::PointXYZRGB point = outliersCloud->points.at(pointIndexr);		
		if ( point.x <= x + 0.0001 && point.x >= x - 0.0001 && point.y <= y + 0.0001 && point.y >= y - 0.0001 && point.z <= z + 0.0001 && point.z >= z - 0.0001)
			{
			found = true;
			int32_t verifyIndex =  visualizedIndicesList.at(pointIndexr);
			ASSERT(verifyIndex == originalPointIndex, "The point does not match the visualization");
			}
		}
	
	//Sometimes the point is not found. Is it a bug of PCL? In that case we ignore the input.
	if (!found)
		{
		return;
		}

	if (outliersBoxActive)
		{
		outliersBox.push_back(originalPointIndex);
		}
	else
		{
		outliersVector.push_back(originalPointIndex);
		}

	cloudChangedSinceLastVisualization = true;
	}

void OutliersDetector::KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event, void* userdata)
	{
	((OutliersDetector*)userdata)->KeyboardButtonCallback(event);
	}

void OutliersDetector::KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event)
	{
	std::lock_guard<std::mutex> guard(inputMutex);

	if (!event.keyDown())
		{
		return;
		}
	unsigned command = static_cast<unsigned>( event.getKeyCode() );

  	if (command == 17) //Ctrl+Q
		{
		detectorIsActive = false;
		}
  	else if (command == 14) //Ctrl+N
		{
		if (outliersBoxActive && outliersBox.size() > 0)
			{
			outliersBox.pop_back();
			cloudChangedSinceLastVisualization = true;
			}
		else if (!outliersBoxActive && outliersVector.size() > 0)
			{
			outliersVector.pop_back();
			cloudChangedSinceLastVisualization = true;	
			}
		return;
		}
	else if (command == 13) //Ctrl+M
		{
		SaveOutliers();
		PRINT_TO_LOG("Data Saved", "");
		}
	else if (command == 23) //Ctrl+W
		{
		visualizationCenter.y += visualizationRadius/2;
		focusChangedSinceLastVisualization = true;
		cloudChangedSinceLastVisualization = true;
		PRINT_CLOUD_LOCATION_INFO("Source", visualizationCenter, visualizationRadius);
		}
	else if (command == 24) //Ctrl+X
		{
		visualizationCenter.y -= visualizationRadius/2;
		focusChangedSinceLastVisualization = true;
		cloudChangedSinceLastVisualization = true;
		PRINT_CLOUD_LOCATION_INFO("Source", visualizationCenter, visualizationRadius);
		}
	else if (command == 1) //Ctrl+A
		{
		visualizationCenter.x -= visualizationRadius/2;
		focusChangedSinceLastVisualization = true;
		cloudChangedSinceLastVisualization = true;
		PRINT_CLOUD_LOCATION_INFO("Source", visualizationCenter, visualizationRadius);
		}
	else if (command == 4) //Ctrl+D
		{
		visualizationCenter.x += visualizationRadius/2;
		focusChangedSinceLastVisualization = true;
		cloudChangedSinceLastVisualization = true;
		PRINT_CLOUD_LOCATION_INFO("Source", visualizationCenter, visualizationRadius);
		}
	else if (command == 5) //Ctrl+E
		{
		visualizationCenter.z += visualizationRadius/2;
		focusChangedSinceLastVisualization = true;
		cloudChangedSinceLastVisualization = true;
		PRINT_CLOUD_LOCATION_INFO("Source", visualizationCenter, visualizationRadius);
		}
	else if (command == 26) //Ctrl+Z
		{
		visualizationCenter.z -= visualizationRadius/2;
		focusChangedSinceLastVisualization = true;
		cloudChangedSinceLastVisualization = true;
		PRINT_CLOUD_LOCATION_INFO("Source", visualizationCenter, visualizationRadius);
		}
	else if (command == 15) //Ctrl+O
		{
		visualizationRadius = visualizationRadius * 1.5;
		focusChangedSinceLastVisualization = true;
		cloudChangedSinceLastVisualization = true;
		PRINT_CLOUD_LOCATION_INFO("Source", visualizationCenter, visualizationRadius);
		}
	else if (command == 16) //Ctrl+P
		{
		visualizationRadius = visualizationRadius / 1.5;
		focusChangedSinceLastVisualization = true;
		cloudChangedSinceLastVisualization = true;
		PRINT_CLOUD_LOCATION_INFO("Source", visualizationCenter, visualizationRadius);
		}	
	else if (command == 6) // Ctrl+F
		{
		outliersBoxActive = true;
		outliersBox.clear();
		PRINT_TO_LOG("Box selection is active", "");
		}
	else if (command == 7) // Ctrl+G
		{
		std::vector<int32_t> addedOutliersList = ComputeOutliersBoxClosure();
		outliersVector.insert(outliersVector.end(), addedOutliersList.begin(), addedOutliersList.end());
		outliersBoxActive = false;
		cloudChangedSinceLastVisualization = true;
		outliersBox.clear();
		PRINT_TO_LOG("Box selection is no longer active", "");
		}
	else if (command == 2) //Ctrl+B
		{
		SaveOutliersAsPointCloud();
		PRINT_TO_LOG("Data Saved", "");
		}
	}

#define COPY_WHITE_POINT(origin, center, displacementX, destination) \
	{ \
	destination.x = origin.x - center.x + displacementX; \
	destination.y = origin.y - center.y; \
	destination.z = origin.z - center.z; \
	destination.r = 255; \
	destination.g = 255; \
	destination.b = 255; \
	}

#define COPY_COLOR(point, color) \
	{ \
	point.r = color[0]; \
	point.g = color[1]; \
	point.b = color[2]; \
	}

void OutliersDetector::PrepareCloudToVisualize()
	{
	std::vector<float> squaredDistancesVector;
	visualizedIndicesList.clear();	

	originalCloudSearchTree.radiusSearch(visualizationCenter, visualizationRadius, visualizedIndicesList, squaredDistancesVector);

	outliersCloud->points.resize(visualizedIndicesList.size());

	focusChangedSinceLastVisualization = false;
	}

void OutliersDetector::DrawOutliers()
	{
	const cv::Scalar& color = COLORS_LIST.at(2);
	const cv::Scalar& boxColor = COLORS_LIST.at(3);
	std::vector<int32_t> boxClosure = ComputeOutliersBoxClosure();

	for(int32_t visualizedPointIndex = 0; visualizedPointIndex < visualizedIndicesList.size(); visualizedPointIndex++)
		{
		int32_t pointIndex = visualizedIndicesList.at(visualizedPointIndex);
		pcl::PointXYZ& point = originalCloud->points.at(pointIndex);
		pcl::PointXYZRGB& visualizedPoint = outliersCloud->points.at(visualizedPointIndex);
		COPY_WHITE_POINT(point, visualizationCenter, 0, visualizedPoint); 

		if (std::find(outliersVector.begin(), outliersVector.end(), pointIndex) != outliersVector.end())
			{
			COPY_COLOR(visualizedPoint, color);
			}

		if (std::find(boxClosure.begin(), boxClosure.end(), pointIndex) != boxClosure.end())
			{
			COPY_COLOR(visualizedPoint, boxColor);
			}
		}

	pointCloudColorHandler = boost::make_shared<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> >(outliersCloud);
	visualizer->updatePointCloud< pcl::PointXYZRGB >(outliersCloud, *pointCloudColorHandler, "outliersCloud");
	}

void OutliersDetector::SaveOutliersAsPointCloud()
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(int outlierIndex = 0; outlierIndex < outliersVector.size(); outlierIndex++)
		{
		int32_t pointIndex = outliersVector.at(outlierIndex);
		pcl::PointXYZ newPoint = originalCloud->points.at(pointIndex);
		pclCloud->points.push_back(newPoint);
		}

	std::stringstream outputFilePathStream;
	outputFilePathStream << outliersFilePath.substr(0, outliersFilePath.size()-4) << ".ply";
	pcl::PLYWriter writer;
	writer.write(outputFilePathStream.str(), *pclCloud, true);
	}

void OutliersDetector::SaveOutliers()
	{
	cv::Mat outliersMatrix(outliersVector.size(), 1, CV_32SC1);
	for(int outlierIndex = 0; outlierIndex < outliersVector.size(); outlierIndex++)
		{
		outliersMatrix.at<int32_t>(outlierIndex, 0) = outliersVector.at(outlierIndex);
		}

	cv::FileStorage opencvFile(outliersFilePath, cv::FileStorage::WRITE);
	opencvFile << "OutliersMatrix" << outliersMatrix;
	opencvFile.release();
	}

void OutliersDetector::LoadOutliers()
	{
	cv::Mat outliersMatrix;
	
	try 
		{
		cv::FileStorage opencvFile(outliersFilePath, cv::FileStorage::READ);
		opencvFile["OutliersMatrix"] >> outliersMatrix;
		opencvFile.release();
		}
	catch (...)
		{
		//If reading fails, just overwrite the file.
		return;
		}

	ASSERT( (outliersMatrix.rows == 0 || (outliersMatrix.type() == CV_32SC1 && outliersMatrix.cols == 1)), "Error in loaded file, invalid format");

	for(int outlierIndex = 0; outlierIndex < outliersVector.size(); outlierIndex++)
		{
		int32_t outlier = outliersMatrix.at<int32_t>(outlierIndex, 0);
		outliersVector.push_back(outlier);
		}
	}

void OutliersDetector::ComputeCloudCenter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointXYZ& center)
	{	
	center.x = 0;
	center.y = 0;
	center.z = 0;
	for(int32_t pointIndex = 0; pointIndex < cloud->points.size(); pointIndex++)
		{
		const pcl::PointXYZ& point = cloud->points.at(pointIndex);
		center.x += point.x;
		center.y += point.y;
		center.z += point.z;
		}
	center.x = (center.x / (float) cloud->points.size());
	center.y = (center.y / (float) cloud->points.size());
	center.z = (center.z / (float) cloud->points.size());
	}

float OutliersDetector::ComputeCloudRadius(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const pcl::PointXYZ& center)
	{
	float radius = 0;
	for(int32_t pointIndex = 0; pointIndex < cloud->points.size(); pointIndex++)
		{
		const pcl::PointXYZ& point = cloud->points.at(pointIndex);
		float differenceX = point.x - center.x;
		float differenceY = point.y - center.y;
		float differenceZ = point.z - center.z;
		float distance = std::sqrt( differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ);
		if (distance > radius)
			{
			radius = distance;
			}
		}
	return radius;
	}

float OutliersDetector::ComputeMinimumDistanceBetweenPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
	{
	float minimumDistance = -1;
	for(int32_t point1Index = 0; point1Index < cloud->points.size()-1; point1Index++)
		{
		const pcl::PointXYZ& point1 = cloud->points.at(point1Index);
		const pcl::PointXYZ& point2 = cloud->points.at(point1Index+1);
		float differenceX = point1.x - point2.x;
		float differenceY = point1.y - point2.y;
		float differenceZ = point1.z - point2.z;
		float distance = std::sqrt( differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ);
		if (minimumDistance < 0 || distance < minimumDistance)
			{
			minimumDistance = distance;
			}		
		}
	return minimumDistance;
	}

std::vector<int32_t> OutliersDetector::ComputeOutliersBoxClosure()
	{
	std::vector<int32_t> closure;
	if (outliersBox.size() == 0)
		{
		return closure;
		}

	float minX, maxX, minY, maxY, minZ, maxZ;
	ComputeBoxBorders(minX, maxX, minY, maxY, minZ, maxZ);

	for(int pointIndex = 0; pointIndex < originalCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = originalCloud->points.at(pointIndex);
		if ( minX <= point.x && point.x <= maxX && minY <= point.y && point.y <= maxY && minZ <= point.z && point.z <= maxZ)
			{
			int copyIndex = pointIndex;
			closure.push_back(copyIndex);
			}
		}
	return closure;
	}

void OutliersDetector::ComputeBoxBorders(float& minX, float& maxX, float& minY, float& maxY, float& minZ, float& maxZ)
	{
	if (outliersBox.size() == 0)
		{
		return;
		}

	pcl::PointXYZ startPoint = originalCloud->points.at( outliersBox.at(0) );
	minX = startPoint.x;
	maxX = startPoint.x;
	minY = startPoint.y;
	maxY = startPoint.y;
	minZ = startPoint.z;
	maxZ = startPoint.z;	

	for(int outlierIndex = 1; outlierIndex < outliersBox.size(); outlierIndex++)
		{
		pcl::PointXYZ point = originalCloud->points.at( outliersBox.at(outlierIndex) );
		if (minX > point.x)
			{
			minX = point.x;
			}
		if (maxX < point.x)
			{
			maxX = point.x;
			}
		if (minY > point.y)
			{
			minY = point.y;
			}
		if (maxY < point.y)
			{
			maxY = point.y;
			}
		if (minZ > point.z)
			{
			minZ = point.z;
			}
		if (maxZ < point.z)
			{
			maxZ = point.z;
			}
		}
	}

}
/** @} */
