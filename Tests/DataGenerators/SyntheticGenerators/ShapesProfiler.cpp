/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ShapesProfiler.cpp
 * @date 15/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the ShapesProfiler class.
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
#include "ShapesProfiler.hpp"
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
ShapesProfiler::ShapesProfiler(std::string inputCloudFilePath, std::string shapesFilePath) :
	originalCloud(new pcl::PointCloud<pcl::PointXYZ>),
	shapesCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
	visualizer(new pcl::visualization::PCLVisualizer("Objects Profiler"))
	{
	this->inputCloudFilePath = inputCloudFilePath;
	this->shapesFilePath = shapesFilePath;

	LoadCloud();
	if (!LoadObjectsAndPoints())
		{
		totalLinesCounter = 0;
		}

	visualizer->registerKeyboardCallback(ShapesProfiler::KeyboardButtonCallback, this);
	visualizer->registerPointPickingCallback(ShapesProfiler::PointPickingCallback, this);

	pointCloudColorHandler = boost::make_shared<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> >(shapesCloud);
	visualizer->addPointCloud< pcl::PointXYZRGB >(shapesCloud, *pointCloudColorHandler, "shapesCloud");

	profilerIsActive = true;
	cloudChangedSinceLastVisualization = true;
	focusChangedSinceLastVisualization = true;
	activeObject = -1;
	profilerMode = PROFILE_DISTANCES_TO_CAMERA;
	profilerState = START_STATE;

	PRINT_TO_LOG("Starting in distance to camera profiling mode", "");
	}

ShapesProfiler::~ShapesProfiler()
	{	
	visualizer->close();
	}

void ShapesProfiler::Run()
	{
	while(profilerIsActive && !visualizer->wasStopped())
		{
		if (cloudChangedSinceLastVisualization)
			{
			std::lock_guard<std::mutex> guard(inputMutex);
			visualizer->removeAllShapes();
			if (focusChangedSinceLastVisualization)
				{
				PrepareCloudToVisualize();
				}
			if (profilerMode == PROFILE_OBJECTS)
				{
				DrawObjects();
				}
			else if (profilerMode == INSPECTION)
				{
				DrawPointsToCamera();
				DrawObjects();
				}
			else
				{
				DrawPointsToCamera();
				}
			VisualizeCloud();
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
const std::vector<cv::Scalar> ShapesProfiler::COLORS_LIST =
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
pcl::PointXYZRGB ShapesProfiler::nonVisualizedPoint;
	


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

void ShapesProfiler::LoadCloud()
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

void ShapesProfiler::PointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* userdata)
	{
	((ShapesProfiler*)userdata)->PointPickingCallback(event);
	}

void ShapesProfiler::PointPickingCallback(const pcl::visualization::PointPickingEvent& event)
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
	for(int pointIndexr = 0; pointIndexr < shapesCloud->points.size() && !found; pointIndexr++)
		{
		pcl::PointXYZRGB point = shapesCloud->points.at(pointIndexr);		
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

	if (profilerMode == PROFILE_OBJECTS)
		{
		HandleObjectsModePointPicking(originalPointIndex);
		}
	else if (profilerMode == PROFILE_DISTANCES_TO_CAMERA)
		{
		HandlePointsModePointPicking(originalPointIndex);
		}
	else 
		{
		HandleInspectionPointPicking(originalPointIndex);
		}
	}

void ShapesProfiler::HandleObjectsModePointPicking(int32_t originalPointIndex)
	{
	if (profilerState == START_STATE)
		{
		Line newLine;
		newLine.sourceIndex = originalPointIndex;
		objectsList.at(activeObject).push_back(newLine);
		profilerState = FIRST_LINE_POINT_PICKED;
		totalLinesCounter++;
		PRINT_TO_LOG("Starting a line at:", originalPointIndex);
		cloudChangedSinceLastVisualization = true;
		}
	else if (profilerState == FIRST_LINE_POINT_PICKED)
		{
		if (originalPointIndex == (objectsList.at(activeObject).end()-1)->sourceIndex)
			{
			PRINT_TO_LOG("The start line point cannot be the same as the end line point", "");
			}
		else
			{
			(objectsList.at(activeObject).end()-1)->sinkIndex = originalPointIndex;
			profilerState = SECOND_LINE_POINT_PICKED;
			PRINT_TO_LOG("line added, ending at:", originalPointIndex);
			}
		cloudChangedSinceLastVisualization = true;
		}
	else
		{
		PRINT_TO_LOG("You already selected a line, press Ctr+J to add a distance, or press Ctr+N to delete the current line", "");
		}
	}

void ShapesProfiler::HandlePointsModePointPicking(int32_t originalPointIndex)
	{
	if (profilerState == START_STATE)
		{
		Point newPoint;
		newPoint.index = originalPointIndex;
		pointsToCameraList.push_back(newPoint);
		profilerState = POINT_TO_CAMERA_PICKED;
		PRINT_TO_LOG("Point added: ", originalPointIndex);
		cloudChangedSinceLastVisualization = true;
		}
	else
		{
		PRINT_TO_LOG("You already selected a point, press Ctr+J to add a point-to-camera distance, or press Ctr+N to delete the current point", "");
		}
	}

void ShapesProfiler::HandleInspectionPointPicking(int32_t originalPointIndex)
	{
	for(int objectIndex = 0; objectIndex < objectsList.size(); objectIndex++)
		{
		Object& currentObject = objectsList.at(objectIndex);
		for(int lineIndex = 0; lineIndex < currentObject.size(); lineIndex++)
			{
			Line& currentLine = currentObject.at(lineIndex);
			if (currentLine.sourceIndex == originalPointIndex || currentLine.sinkIndex == originalPointIndex)
				{
				std::stringstream lineStream;
				lineStream << "Object: " << objectIndex << " Line: " << lineIndex << " length: " << currentLine.length;
				PRINT_TO_LOG(lineStream.str(), "");
				}
			}
		}

	for(int pointIndex = 0; pointIndex < pointsToCameraList.size(); pointIndex++)
		{
		Point& currentPoint = pointsToCameraList.at(pointIndex);
		if (currentPoint.index == originalPointIndex)
			{
			PRINT_TO_LOG("Distance of the point from the camera is: ", currentPoint.distanceToCamera);
			}
		}
	}

void ShapesProfiler::KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event, void* userdata)
	{
	((ShapesProfiler*)userdata)->KeyboardButtonCallback(event);
	}

void ShapesProfiler::KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event)
	{
	std::lock_guard<std::mutex> guard(inputMutex);

	if (!event.keyDown())
		{
		return;
		}
	unsigned command = static_cast<unsigned>( event.getKeyCode() );

  	if (command == 17) //Ctrl+Q
		{
		profilerIsActive = false;
		}
  	else if (command == 14) //Ctrl+N
		{
		if (profilerMode == PROFILE_OBJECTS && objectsList.at(activeObject).size() > 0)
			{
			objectsList.at(activeObject).pop_back();
			cloudChangedSinceLastVisualization = true;
			totalLinesCounter--;
			profilerState = START_STATE;
			focusChangedSinceLastVisualization = true;
			}
		else if (pointsToCameraList.size() > 0)
			{
			pointsToCameraList.pop_back();
			cloudChangedSinceLastVisualization = true;
			profilerState = START_STATE;
			focusChangedSinceLastVisualization = true;
			}
		return;
		}
	else if (command == 13) //Ctrl+M
		{
		SaveObjectsAndPoints();
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
	else if (command == 25) // Ctrl+Y
		{
		if (profilerMode == PROFILE_OBJECTS && profilerState == START_STATE)
			{
			profilerMode = PROFILE_DISTANCES_TO_CAMERA;
			PRINT_TO_LOG("Switching to point to camera profiling", "");
			focusChangedSinceLastVisualization = true;
			cloudChangedSinceLastVisualization = true;
			}
		else if (profilerState == START_STATE)
			{
			profilerMode = PROFILE_OBJECTS;
			if (objectsList.size() == 0 || activeObject == -1)
				{
				Object newObject;
				objectsList.push_back(newObject);
				activeObject = objectsList.size() - 1;
				}
			PRINT_TO_LOG("Switching to object profiling, current object is Number", activeObject);
			focusChangedSinceLastVisualization = true;
			cloudChangedSinceLastVisualization = true;
			} 
		}
	else if (command == 11) // Ctrl+K
		{
		if (profilerMode == PROFILE_DISTANCES_TO_CAMERA)
			{
			return;
			}
		if (activeObject > 0)
			{
			if (activeObject == objectsList.size()-1 && objectsList.at(activeObject).size() == 0)
				{
				objectsList.pop_back();
				}			
			activeObject--;
			PRINT_TO_LOG("Moving to active object Number", activeObject);
			}
		}
	else if (command == 12) // Ctrl+L
		{
		if (profilerMode == PROFILE_DISTANCES_TO_CAMERA)
			{
			return;
			}
		if (activeObject == objectsList.size()-1)
			{
			Object newObject;
			objectsList.push_back(newObject);
			}
		activeObject++;
		PRINT_TO_LOG("Operating on object N.", activeObject);
		}
	else if (command == 10) // Ctrl+J
		{
		float distance;
		if (profilerState == POINT_TO_CAMERA_PICKED)
			{
			std::cout << "Please input the distance of the selected point to the camera:" <<std::endl;
			cin >> distance;
			(pointsToCameraList.end()-1)->distanceToCamera = distance;
			profilerState = START_STATE;
			cloudChangedSinceLastVisualization = true;
			}
		else if (profilerState == SECOND_LINE_POINT_PICKED)
			{
			std::cout << "Please input the length of the selected line:" <<std::endl;
			cin >> distance;
			(objectsList.at(activeObject).end()-1)->length = distance;
			profilerState = START_STATE;
			cloudChangedSinceLastVisualization = true;
			}
		}
	else if (command == 2) //Ctrl+B
		{
		if (profilerState == START_STATE && profilerMode != INSPECTION)
			{
			profilerMode = INSPECTION;
			focusChangedSinceLastVisualization = true;
			cloudChangedSinceLastVisualization = true;
			PRINT_TO_LOG("Switching to point to inspection mode", "");
			}
		else if (profilerMode == INSPECTION)
			{
			profilerMode = PROFILE_DISTANCES_TO_CAMERA;
			PRINT_TO_LOG("Switching to point to camera profiling", "");
			focusChangedSinceLastVisualization = true;
			cloudChangedSinceLastVisualization = true;
			}
		else
			{
			PRINT_TO_LOG("plese complete your selection before switching to inspection state", "");
			}
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

void ShapesProfiler::PrepareCloudToVisualize()
	{
	std::vector<float> squaredDistancesVector;
	visualizedIndicesList.clear();	

	originalCloudSearchTree.radiusSearch(visualizationCenter, visualizationRadius, visualizedIndicesList, squaredDistancesVector);

	shapesCloud->points.resize(visualizedIndicesList.size());

	for(int32_t visualizedPointIndex = 0; visualizedPointIndex < visualizedIndicesList.size(); visualizedPointIndex++)
		{
		int32_t pointIndex = visualizedIndicesList.at(visualizedPointIndex);
		pcl::PointXYZ& point = originalCloud->points.at(pointIndex);
		pcl::PointXYZRGB& visualizedPoint = shapesCloud->points.at(visualizedPointIndex);
		COPY_WHITE_POINT(point, visualizationCenter, 0, visualizedPoint); 
		}

	focusChangedSinceLastVisualization = false;
	}

void ShapesProfiler::DrawObjects()
	{
	const int RED_COLOR_INDEX = 3;
	int nextColorIndex = -1;
	for(int objectIndex = 0; objectIndex < objectsList.size(); objectIndex++)
		{
		nextColorIndex = (nextColorIndex == RED_COLOR_INDEX-1) ? RED_COLOR_INDEX+1 : (nextColorIndex+1)%COLORS_LIST.size();
		const cv::Scalar color = COLORS_LIST.at( nextColorIndex ); 

		DrawObject(objectIndex, color);
		}
	}

void ShapesProfiler::DrawObject(int objectIndex, const cv::Scalar& color)
	{
	static const cv::Scalar& RED_COLOR = cv::Scalar(255, 0, 0);
	Object& currentObject = objectsList.at(objectIndex);
	for(int lineIndex = 0; lineIndex < currentObject.size(); lineIndex++)
		{
		if (objectIndex == activeObject && lineIndex == currentObject.size()-1 && profilerState == FIRST_LINE_POINT_PICKED)
			{
			continue;
			} 
		Line& currentLine = currentObject.at(lineIndex);
		bool sourceIsVisualized, sinkIsVisualized;
		pcl::PointXYZRGB& visualizedSourcePoint = GetVisualizedPoint(currentLine.sourceIndex, sourceIsVisualized);
		pcl::PointXYZRGB& visualizedSinkPoint = GetVisualizedPoint(currentLine.sinkIndex, sinkIsVisualized);
		if (sourceIsVisualized && sinkIsVisualized)
			{
			std::stringstream lineName;
			lineName << "line_" << objectIndex << "_" << lineIndex; 
			if (objectIndex == activeObject && lineIndex == currentObject.size()-1 && profilerState == SECOND_LINE_POINT_PICKED)
				{
				visualizer->addLine(visualizedSourcePoint, visualizedSinkPoint, RED_COLOR[0], RED_COLOR[1], RED_COLOR[2], lineName.str());
				}
			else
				{
				visualizer->addLine(visualizedSourcePoint, visualizedSinkPoint, color[0], color[1], color[2], lineName.str());
				}
			}
		}

	if (objectIndex == activeObject && currentObject.size() > 0 && profilerState == FIRST_LINE_POINT_PICKED)
		{
		Line& currentLine = currentObject.at(currentObject.size()-1);
		bool sourceIsVisualized;
		pcl::PointXYZRGB& visualizedSourcePoint = GetVisualizedPoint(currentLine.sourceIndex, sourceIsVisualized);
		if (sourceIsVisualized)
			{
			COPY_COLOR(visualizedSourcePoint, RED_COLOR);
			}
		}
	}

pcl::PointXYZRGB& ShapesProfiler::GetVisualizedPoint(int32_t originalPointIndex, bool& pointIsVisualized)
	{
	std::vector<int32_t>::iterator originalPointIndexInVisualizedList = std::find(visualizedIndicesList.begin(), visualizedIndicesList.end(), originalPointIndex);
	if (originalPointIndexInVisualizedList != visualizedIndicesList.end())
		{
		int32_t visualizedPointIndex = originalPointIndexInVisualizedList - visualizedIndicesList.begin();
		pointIsVisualized = true;
		return shapesCloud->points.at(visualizedPointIndex);
		}
	
	pointIsVisualized = false;
	return nonVisualizedPoint;
	}

void ShapesProfiler::VisualizeCloud()
	{
	pointCloudColorHandler = boost::make_shared<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> >(shapesCloud);
	visualizer->updatePointCloud< pcl::PointXYZRGB >(shapesCloud, *pointCloudColorHandler, "shapesCloud");
	}

void ShapesProfiler::DrawPointsToCamera()
	{
	static const cv::Scalar RED_COLOR = cv::Scalar(255, 0, 0);
	static const cv::Scalar GREEN_COLOR = cv::Scalar(0, 255, 255);

	for(int pointIndex = 0; pointIndex < pointsToCameraList.size(); pointIndex++)
		{
		Point& currentPoint = pointsToCameraList.at(pointIndex);
		bool pointIsVisualized;
		pcl::PointXYZRGB& visualizedPoint = GetVisualizedPoint(currentPoint.index, pointIsVisualized);
		
		if (pointIsVisualized)
			{
			std::stringstream sphereName;
			sphereName << "sphere_" << pointIndex; 
			if (pointIndex == pointsToCameraList.size() -1 && profilerState != START_STATE)
				{
				COPY_COLOR(visualizedPoint, RED_COLOR);
				visualizer->addSphere(visualizedPoint, visualizationCircleSize, RED_COLOR[0], RED_COLOR[1], RED_COLOR[2], sphereName.str());
				}
			else
				{
				COPY_COLOR(visualizedPoint, GREEN_COLOR);
				visualizer->addSphere(visualizedPoint, visualizationCircleSize, GREEN_COLOR[0], GREEN_COLOR[1], GREEN_COLOR[2], sphereName.str());
				}
			}
		}
	}

void ShapesProfiler::SaveObjectsAndPoints()
	{
	cv::Mat objectsMatrix(totalLinesCounter, 4, CV_32FC1);
	int lineCounter = 0;
	for(int objectIndex = 0; objectIndex < objectsList.size(); objectIndex++)
		{
		Object& currentObject = objectsList.at(objectIndex);
		for(int lineIndex = 0; lineIndex < currentObject.size(); lineIndex++)
			{
			if (lineIndex == currentObject.size()-1 && (profilerState == FIRST_LINE_POINT_PICKED || profilerState == SECOND_LINE_POINT_PICKED) )
				{
				continue;
				} 
			Line& currentLine = currentObject.at(lineIndex);
			objectsMatrix.at<float>(lineCounter, 0) = currentLine.sourceIndex;
			objectsMatrix.at<float>(lineCounter, 1) = currentLine.sinkIndex;
			objectsMatrix.at<float>(lineCounter, 2) = currentLine.length;
			objectsMatrix.at<float>(lineCounter, 3) = objectIndex;
			lineCounter++;
			}
		}		

	cv::Mat pointsToCameraMatrix(pointsToCameraList.size(), 2, CV_32FC1);
	for(int pointIndex = 0; pointIndex < pointsToCameraList.size(); pointIndex++)
		{
		if (pointIndex == pointsToCameraList.size()-1 && profilerState == POINT_TO_CAMERA_PICKED)
			{
			continue;
			}
		Point& currentPoint = pointsToCameraList.at(pointIndex);
		pointsToCameraMatrix.at<float>(pointIndex, 0) = currentPoint.index;
		pointsToCameraMatrix.at<float>(pointIndex, 1) = currentPoint.distanceToCamera;
		}	

	cv::FileStorage opencvFile(shapesFilePath, cv::FileStorage::WRITE);
	opencvFile << "ObjectsMatrix" << objectsMatrix;
	opencvFile << "PointsToCameraMatrix" << pointsToCameraMatrix;
	opencvFile.release();
	}

bool ShapesProfiler::LoadObjectsAndPoints()
	{
	cv::Mat objectsMatrix, pointsToCameraMatrix;
	
	try 
		{
		cv::FileStorage opencvFile(shapesFilePath, cv::FileStorage::READ);
		opencvFile["ObjectsMatrix"] >> objectsMatrix;
		opencvFile["PointsToCameraMatrix"] >> pointsToCameraMatrix;
		opencvFile.release();
		}
	catch (...)
		{
		//If reading fails, just overwrite the file.
		return false;
		}

	ASSERT( (objectsMatrix.rows == 0 || (objectsMatrix.type() == CV_32FC1 && objectsMatrix.cols == 4)), "Error in loaded file, invalid format");
	ASSERT( (pointsToCameraMatrix.rows == 0 || (pointsToCameraMatrix.type() == CV_32FC1 && pointsToCameraMatrix.cols == 2)), "Error in loaded file, invalid format");

	for(int pointIndex = 0; pointIndex < pointsToCameraMatrix.rows; pointIndex++)
		{
		Point newPoint;
		newPoint.index = pointsToCameraMatrix.at<float>(pointIndex, 0);
		newPoint.distanceToCamera = pointsToCameraMatrix.at<float>(pointIndex, 1);
		pointsToCameraList.push_back(newPoint);
		}

	for(int lineIndex = 0; lineIndex < objectsMatrix.rows; lineIndex++)
		{
		int objectIndex = objectsMatrix.at<float>(lineIndex, 3);
		while(objectIndex >= objectsList.size())
			{
			Object newObject;
			objectsList.push_back(newObject);
			}

		Line newLine;
		newLine.sourceIndex = objectsMatrix.at<float>(lineIndex, 0);
		newLine.sinkIndex = objectsMatrix.at<float>(lineIndex, 1);
		newLine.length = objectsMatrix.at<float>(lineIndex, 2);
		objectsList.at(objectIndex).push_back(newLine);
		}
	totalLinesCounter = objectsMatrix.rows;
	return true;
	}

void ShapesProfiler::ComputeCloudCenter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointXYZ& center)
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

float ShapesProfiler::ComputeCloudRadius(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const pcl::PointXYZ& center)
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

float ShapesProfiler::ComputeMinimumDistanceBetweenPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
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

}
/** @} */
