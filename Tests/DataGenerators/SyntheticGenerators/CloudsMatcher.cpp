/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CloudsMatcher.cpp
 * @date 10/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the CloudsMatcher class.
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
#include "CloudsMatcher.hpp"
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <Errors/Assert.hpp>

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
CloudsMatcher::CloudsMatcher(std::string inputSourceCloudFilePath, std::string inputSinkCloudFilePath, std::string outputCorrespondencesFilePath) :
	originalSourceCloud(new pcl::PointCloud<pcl::PointXYZ>),
	originalSinkCloud(new pcl::PointCloud<pcl::PointXYZ>),
	correspondencesCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
	visualizer(new pcl::visualization::PCLVisualizer("Clouds Matcher"))
	{
	this->inputSourceCloudFilePath = inputSourceCloudFilePath;
	this->inputSinkCloudFilePath = inputSinkCloudFilePath;
	this->outputCorrespondencesFilePath = outputCorrespondencesFilePath; 

	LoadClouds();
	LoadCorrespondences();

	visualizer->registerKeyboardCallback(CloudsMatcher::KeyboardButtonCallback, this);
	visualizer->registerPointPickingCallback(CloudsMatcher::PointPickingCallback, this);

	//pointCloudColorHandler = boost::make_shared< pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> >(correspondencesCloud);
	visualizer->addPointCloud< pcl::PointXYZRGB >(correspondencesCloud, "correspondencesCloud");

	matcherIsActive = true;
	cloudsChangedSinceLastVisualization = true;
	}

CloudsMatcher::~CloudsMatcher()
	{	
	visualizer->close();
	}

void CloudsMatcher::Run()
	{
	while(matcherIsActive && !visualizer->wasStopped())
		{
		if (cloudsChangedSinceLastVisualization)
			{
			std::lock_guard<std::mutex> guard(inputMutex);
			PrepareCloudsToVisualize();
			DrawCorrespondencesCloud();
			cloudsChangedSinceLastVisualization = false;
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
const std::vector<cv::Scalar> CloudsMatcher::COLORS_LIST =
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
void CloudsMatcher::LoadClouds()
	{
	pcl::io::loadPLYFile(inputSourceCloudFilePath, *originalSourceCloud);
	originalSourceCloudSearchTree.setInputCloud(originalSourceCloud);
	PRINT_TO_LOG("Number of points in source cloud: ", (originalSourceCloud->points.size()) );
	ComputeCloudCenter(originalSourceCloud, sourceVisualizationCenter);
	sourceVisualizationRadius = ComputeCloudRadius(originalSourceCloud, sourceVisualizationCenter) / 10;

	pcl::io::loadPLYFile(inputSinkCloudFilePath, *originalSinkCloud);
	originalSinkCloudSearchTree.setInputCloud(originalSinkCloud);
	PRINT_TO_LOG("Number of points in sink cloud: ", (originalSinkCloud->points.size()) );
	ComputeCloudCenter(originalSinkCloud, sinkVisualizationCenter);
	sinkVisualizationRadius = ComputeCloudRadius(originalSinkCloud, sinkVisualizationCenter) / 10;

	float sourceMinimumDistance = ComputeMinimumDistanceBetweenPoints(originalSourceCloud);
	float sinkMinimumDistance = ComputeMinimumDistanceBetweenPoints(originalSinkCloud);
	visualizationCircleSize = (sourceMinimumDistance > sinkMinimumDistance) ? sinkMinimumDistance/2 : sourceMinimumDistance/2;
	}

void CloudsMatcher::PointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* userdata)
	{
	((CloudsMatcher*)userdata)->PointPickingCallback(event);
	}

void CloudsMatcher::PointPickingCallback(const pcl::visualization::PointPickingEvent& event)
	{
	int32_t pointIndex = event.getPointIndex();
	if(pointIndex == -1)
		{
		return;
		}

	//The type of selection depends on whether the point index is within the source indices or within the sink indices.
	SelectionType currentSelection = (pointIndex < visualizedSourceIndicesList.size() ? SELECTED_FROM_SOURCE : SELECTED_FROM_SINK);
	cloudsChangedSinceLastVisualization = true;	

	//Computes the effective point index on the original clouds.
	int32_t originalPointIndex;
	if (currentSelection == SELECTED_FROM_SOURCE)
		{
		originalPointIndex = visualizedSourceIndicesList.at(pointIndex);
		}
	else
		{
		originalPointIndex = visualizedSinkIndicesList.at(pointIndex - visualizedSourceIndicesList.size() );
		}

	//Add one selected point at the bottom of the correspondencesVector in case the vector is empty or the last correspondence is complete.
	if (correspondencesVector.size() == 0 || (correspondencesVector.end()-1)->selection == SELECTED_FROM_BOTH)
		{
		Correspondence correspondence;
		if (currentSelection == SELECTED_FROM_SOURCE)
			{
			correspondence.sourceIndex = originalPointIndex;
			}
		else
			{
			correspondence.sinkIndex = originalPointIndex;
			}
		correspondence.selection = currentSelection;
		correspondencesVector.push_back(correspondence);
		return;
		}

	//If the last correspondence is not complete, we add the corresponding point to the previous selection.
	std::vector<Correspondence>::iterator lastEntry = correspondencesVector.end()-1;
	if (lastEntry->selection == SELECTED_FROM_SOURCE && currentSelection == SELECTED_FROM_SINK)
		{
		lastEntry->sinkIndex = originalPointIndex;
		lastEntry->selection = SELECTED_FROM_BOTH;
		}
	else if (lastEntry->selection == SELECTED_FROM_SINK && currentSelection == SELECTED_FROM_SOURCE)
		{
		lastEntry->sourceIndex = originalPointIndex;
		lastEntry->selection = SELECTED_FROM_BOTH;
		}
	}

void CloudsMatcher::KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event, void* userdata)
	{
	((CloudsMatcher*)userdata)->KeyboardButtonCallback(event);
	}

void CloudsMatcher::KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event)
	{
	if (!event.keyDown())
		{
		return;
		}
	unsigned command = static_cast<unsigned>( event.getKeyCode() );

	std::lock_guard<std::mutex> guard(inputMutex);
  	if (command == 17) //Ctrl+Q
		{
		matcherIsActive = false;
		}
  	else if (command == 14) //Ctrl+N
		{
		if (correspondencesVector.size() > 0)
			{
			correspondencesVector.pop_back();
			cloudsChangedSinceLastVisualization = true;	
			}
		return;
		}
	else if (command == 13) //Ctrl+M
		{
		SaveCorrespondences();
		}
	else if (command == 23) //Ctrl+W
		{
		sourceVisualizationCenter.y += sourceVisualizationRadius/2;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 24) //Ctrl+X
		{
		sourceVisualizationCenter.y -= sourceVisualizationRadius/2;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 1) //Ctrl+A
		{
		sourceVisualizationCenter.x -= sourceVisualizationRadius/2;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 4) //Ctrl+D
		{
		sourceVisualizationCenter.x += sourceVisualizationRadius/2;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 5) //Ctrl+E
		{
		sourceVisualizationCenter.z += sourceVisualizationRadius/2;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 26) //Ctrl+Z
		{
		sourceVisualizationCenter.z -= sourceVisualizationRadius/2;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 20) //Ctrl+T
		{
		sinkVisualizationCenter.y += sinkVisualizationRadius/2;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 2) //Ctrl+B
		{
		sinkVisualizationCenter.y -= sinkVisualizationRadius/2;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 6) //Ctrl+F
		{
		sinkVisualizationCenter.x -= sinkVisualizationRadius/2;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 8) //Ctrl+H
		{
		sinkVisualizationCenter.x += sinkVisualizationRadius/2;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 25) //Ctrl+Y
		{
		sinkVisualizationCenter.z += sinkVisualizationRadius/2;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 22) //Ctrl+V
		{
		sinkVisualizationCenter.z -= sinkVisualizationRadius/2;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 15) //Ctrl+O
		{
		sourceVisualizationRadius = sourceVisualizationRadius * 1.5;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 16) //Ctrl+P
		{
		sourceVisualizationRadius = sourceVisualizationRadius / 1.5;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 11) //Ctrl+K
		{
		sinkVisualizationRadius = sinkVisualizationRadius * 1.5;
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 12) //Ctrl+L
		{
		sinkVisualizationRadius = sinkVisualizationRadius / 1.5;
		cloudsChangedSinceLastVisualization = true;
		}	
	}

void CloudsMatcher::PrepareCloudsToVisualize()
	{
	std::vector<float> squaredDistancesVector;
	visualizedSourceIndicesList.clear();	
	visualizedSinkIndicesList.clear();	

	originalSourceCloudSearchTree.radiusSearch(sourceVisualizationCenter, sourceVisualizationRadius, visualizedSourceIndicesList, squaredDistancesVector);
	originalSinkCloudSearchTree.radiusSearch(sinkVisualizationCenter, sinkVisualizationRadius, visualizedSinkIndicesList, squaredDistancesVector);
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

#define COPY_COLOR(color, point) \
	{ \
	point.r = color[0]; \
	point.g = color[1]; \
	point.b = color[2]; \
	}

void CloudsMatcher::DrawCorrespondencesCloud()
	{
	correspondencesCloud->points.resize(visualizedSourceIndicesList.size() + visualizedSinkIndicesList.size());

	for(int32_t visualizedPointIndex = 0; visualizedPointIndex < visualizedSourceIndicesList.size(); visualizedPointIndex++)
		{
		int32_t sourcePointIndex = visualizedSourceIndicesList.at(visualizedPointIndex);
		pcl::PointXYZ& sourcePoint = originalSourceCloud->points.at(sourcePointIndex);
		pcl::PointXYZRGB& visualizedPoint = correspondencesCloud->points.at(visualizedPointIndex);
		COPY_WHITE_POINT(sourcePoint, sourceVisualizationCenter, 0, visualizedPoint); 
		}

	for(int32_t visualizedPointIndex = visualizedSourceIndicesList.size(); visualizedPointIndex < correspondencesCloud->points.size(); visualizedPointIndex++)
		{
		int32_t sinkPointIndex = visualizedSinkIndicesList.at(visualizedPointIndex - visualizedSourceIndicesList.size());
		pcl::PointXYZ& sinkPoint = originalSinkCloud->points.at(sinkPointIndex);
		pcl::PointXYZRGB& visualizedPoint = correspondencesCloud->points.at(visualizedPointIndex);
		COPY_WHITE_POINT(sinkPoint, sinkVisualizationCenter, (sinkVisualizationRadius + sourceVisualizationRadius)*(11/10), visualizedPoint); 
		}

	visualizer->removeAllShapes();
	int nextColor = 0;
	for(int correspondenceIndex = 0; correspondenceIndex < correspondencesVector.size() && correspondencesVector.at(correspondenceIndex).selection == SELECTED_FROM_BOTH; correspondenceIndex++)
		{
		Correspondence& currentCorrespondence = correspondencesVector.at(correspondenceIndex);

		std::vector<int32_t>::iterator sourcePointIndex = std::find(visualizedSourceIndicesList.begin(), visualizedSourceIndicesList.end(), currentCorrespondence.sourceIndex);
		bool sourcePointIsVisible = (sourcePointIndex != visualizedSourceIndicesList.end() );

		std::vector<int32_t>::iterator sinkPointIndex = std::find(visualizedSinkIndicesList.begin(), visualizedSinkIndicesList.end(), currentCorrespondence.sinkIndex);
		bool sinkPointIsVisible = (sinkPointIndex != visualizedSinkIndicesList.end() );

		if (!sourcePointIsVisible || !sinkPointIsVisible)
			{
			continue;
			}

		int32_t sourceVisualizedPointIndex = sourcePointIndex - visualizedSourceIndicesList.begin();
		int32_t sinkVisualizedPointIndex = static_cast<int32_t>(sinkPointIndex - visualizedSinkIndicesList.begin()) + visualizedSourceIndicesList.size();

		pcl::PointXYZRGB& sourcePoint = correspondencesCloud->points.at( sourceVisualizedPointIndex );
		pcl::PointXYZRGB& sinkPoint = correspondencesCloud->points.at( sinkVisualizedPointIndex );
		
		const cv::Scalar& color = COLORS_LIST.at(nextColor);
		COPY_COLOR(color, sourcePoint);
		COPY_COLOR(color, sinkPoint);
		nextColor = (nextColor + 1) % COLORS_LIST.size();
		
		std::stringstream lineName;
		lineName << "line_" << correspondenceIndex;
		visualizer->addLine(sourcePoint, sinkPoint, color[0], color[1], color[2], lineName.str());
		}

	if (correspondencesVector.size() > 0)
		{
		const cv::Scalar& color = COLORS_LIST.at(nextColor);
		std::vector<Correspondence>::iterator lastEntry = correspondencesVector.end()-1;
		if (lastEntry->selection == SELECTED_FROM_SOURCE)
			{
			std::vector<int32_t>::iterator sourcePointIndex = std::find(visualizedSourceIndicesList.begin(), visualizedSourceIndicesList.end(), lastEntry->sourceIndex);
			bool sourcePointIsVisible = (sourcePointIndex != visualizedSourceIndicesList.end() );
			if (sourcePointIsVisible)
				{
				int32_t sourceVisualizedPointIndex = sourcePointIndex - visualizedSourceIndicesList.begin();
				pcl::PointXYZRGB& sourcePoint = correspondencesCloud->points.at( sourceVisualizedPointIndex );
				COPY_COLOR(color, sourcePoint);
				visualizer->addSphere(sourcePoint, visualizationCircleSize, color[0], color[1], color[2], "source_sphere");
				}
			}
		else if (lastEntry->selection == SELECTED_FROM_SINK)
			{
			std::vector<int32_t>::iterator sinkPointIndex = std::find(visualizedSinkIndicesList.begin(), visualizedSinkIndicesList.end(), lastEntry->sinkIndex);
			bool sinkPointIsVisible = (sinkPointIndex != visualizedSinkIndicesList.end() );
			if (sinkPointIsVisible)
				{
				int32_t sinkVisualizedPointIndex = static_cast<int32_t>(sinkPointIndex - visualizedSinkIndicesList.begin()) + visualizedSourceIndicesList.size();
				pcl::PointXYZRGB& sinkPoint = correspondencesCloud->points.at( sinkVisualizedPointIndex );
				COPY_COLOR(color, sinkPoint);
				visualizer->addSphere(sinkPoint, visualizationCircleSize, color[0], color[1], color[2], "sink_sphere");
				}
			}
		}	

	visualizer->updatePointCloud< pcl::PointXYZRGB >(correspondencesCloud, "correspondencesCloud");
	}

void CloudsMatcher::SaveCorrespondences()
	{
	int validCorrespondences = correspondencesVector.size();
	if ( validCorrespondences > 0 && (correspondencesVector.end()-1)->selection != SELECTED_FROM_BOTH)
		{
		validCorrespondences--;
		}

	cv::Mat correspondenceMatrix( validCorrespondences, 2, CV_32SC1);
	for(int row = 0; row < correspondenceMatrix.rows; row++)
		{
		correspondenceMatrix.at<int32_t>(row, 0) = correspondencesVector.at(row).sourceIndex;
		correspondenceMatrix.at<int32_t>(row, 1) = correspondencesVector.at(row).sinkIndex;
		}

	cv::FileStorage opencvFile(outputCorrespondencesFilePath, cv::FileStorage::WRITE);
	opencvFile << "CorrespondenceMap" << correspondenceMatrix;
	opencvFile.release();
	}

void CloudsMatcher::LoadCorrespondences()
	{
	cv::Mat correspondenceMatrix;

	try 
		{
		cv::FileStorage opencvFile(outputCorrespondencesFilePath, cv::FileStorage::READ);
		opencvFile["CorrespondenceMap"] >> correspondenceMatrix;
		opencvFile.release();
		}
	catch (...)
		{
		//If reading fails, just overwrite the file.
		return;
		}

	ASSERT( (correspondenceMatrix.rows == 0 || (correspondenceMatrix.type() == CV_32SC1 && correspondenceMatrix.cols == 2)), "Error in loaded file, invalid format");
	for(int row = 0; row < correspondenceMatrix.rows; row++)
		{
		Correspondence newCorrespondence;
		newCorrespondence.sourceIndex = correspondenceMatrix.at<int32_t>(row, 0);
		newCorrespondence.sinkIndex = correspondenceMatrix.at<int32_t>(row, 1);
		newCorrespondence.selection = SELECTED_FROM_BOTH;
		correspondencesVector.push_back(newCorrespondence);
		}
	}

void CloudsMatcher::ComputeCloudCenter(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointXYZ& center)
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

float CloudsMatcher::ComputeCloudRadius(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const pcl::PointXYZ& center)
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

float CloudsMatcher::ComputeMinimumDistanceBetweenPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
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
