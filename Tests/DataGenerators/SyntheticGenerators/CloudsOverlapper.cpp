/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CloudsOverlapper.cpp
 * @date 10/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the CloudsOverlapper class.
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
#include "CloudsOverlapper.hpp"
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <Errors/Assert.hpp>

#define DELETE_IF_NOT_NULL(pointer) \
	if (pointer != NULL) \
		{ \
		delete(pointer); \
		}

using namespace PoseWrapper;
	

namespace DataGenerators
{

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
CloudsOverlapper::CloudsOverlapper(std::string inputSceneCloudFilePath, std::string inputModelCloudFilePath) :
	visualizationCircleRadius(0.003),
	originalSceneCloud(new pcl::PointCloud<pcl::PointXYZ>),
	originalModelCloud(new pcl::PointCloud<pcl::PointXYZ>),
	transformedModelCloud(new pcl::PointCloud<pcl::PointXYZ>),
	visualizer(new pcl::visualization::PCLVisualizer("Clouds Overlapper")),
	sceneCloudColor(originalSceneCloud, 255, 255, 255),
	modelCloudColor(transformedModelCloud, 255, 0, 0)
	{
	this->inputSceneCloudFilePath = inputSceneCloudFilePath;
	this->inputModelCloudFilePath = inputModelCloudFilePath;

	LoadClouds();

	visualizer->registerKeyboardCallback(CloudsOverlapper::KeyboardButtonCallback, this);
	visualizer->registerPointPickingCallback(CloudsOverlapper::PointPickingCallback, this);

	visualizer->addPointCloud< pcl::PointXYZ >(originalSceneCloud, sceneCloudColor, "sceneCloud");
	visualizer->addPointCloud< pcl::PointXYZ >(transformedModelCloud, modelCloudColor, "modelCloud");

	matcherIsActive = true;
	cloudsChangedSinceLastVisualization = true;
	translationResolution = 0.01;
	rotationResolution = 0.1;

	allPointsSelected = false;
	modelPoseComputed = false;
	for(int pointSelectionIndex = 0; pointSelectionIndex < NumberOfPoints; pointSelectionIndex++)
		{
		pointsInSceneSelected[pointSelectionIndex] = false;
		pointsInModelSelected[pointSelectionIndex] = false;
		}
	}

CloudsOverlapper::~CloudsOverlapper()
	{	
	visualizer->close();
	}

void CloudsOverlapper::Run()
	{
	while(matcherIsActive && !visualizer->wasStopped())
		{
		if (cloudsChangedSinceLastVisualization)
			{
			std::lock_guard<std::mutex> guard(inputMutex);
			VisualizePointClouds();
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
const std::vector<cv::Scalar> CloudsOverlapper::COLORS_LIST =
	{
	cv::Scalar(0, 0, 255),
	cv::Scalar(0, 255, 0),
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
	cv::Scalar(128, 0, 128),
	cv::Scalar(255, 0, 0)
	};


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */

void CloudsOverlapper::LoadClouds()
	{
	pcl::io::loadPLYFile(inputSceneCloudFilePath, *originalSceneCloud);
	pcl::io::loadPLYFile(inputModelCloudFilePath, *originalModelCloud);
	transformedModelCloud->points.resize(originalModelCloud->points.size());

	SetPosition(modelPose, 0, 0, 0);
	SetOrientation(modelPose, 0, 0, 0, 1);

	visualizer->updatePointCloud< pcl::PointXYZ >(originalSceneCloud, sceneCloudColor, "sceneCloud");
	}

void CloudsOverlapper::PointPickingCallback(const pcl::visualization::PointPickingEvent& event, void* userdata)
	{
	((CloudsOverlapper*)userdata)->PointPickingCallback(event);
	}

void CloudsOverlapper::PointPickingCallback(const pcl::visualization::PointPickingEvent& event)
	{
	int32_t pointIndex = event.getPointIndex();
	if(pointIndex == -1)
		{
		return;
		}
	if (allPointsSelected)
		{
		PRINT_TO_LOG("all points already selected", "");
		return;
		}

	float x, y, z;
	event.getPoint(x, y, z);

	bool pointInScene = false;
	int numberOfScenePoints = originalSceneCloud->points.size();
	for(int pointIndex = 0; pointIndex < numberOfScenePoints && !pointInScene; pointIndex++)
		{
		pcl::PointXYZ point = originalSceneCloud->points.at(pointIndex);
		if (point.x == x && point.y == y && point.z == z)
			{
			pointInScene = true;
			}
		}

	for(int pointSelectionIndex = 0; pointSelectionIndex < NumberOfPoints; pointSelectionIndex++)
		{

		if (pointInScene && pointsInSceneSelected[pointSelectionIndex] && !pointsInModelSelected[pointSelectionIndex])
			{
			PRINT_TO_LOG("Scene point was already selected, please select the corresponding point in the model", "");
			return;
			}
		if (!pointInScene && !pointsInSceneSelected[pointSelectionIndex] && pointsInModelSelected[pointSelectionIndex])
			{
			PRINT_TO_LOG("Model point was already selected, please select the corresponding point in the scene", "");
			return;
			}

		if (pointInScene && !pointsInSceneSelected[pointSelectionIndex])
			{
			pointsInSceneSelected[pointSelectionIndex] = true;
			pointsInScene[pointSelectionIndex].x = x;
			pointsInScene[pointSelectionIndex].y = y;
			pointsInScene[pointSelectionIndex].z = z;
			cloudsChangedSinceLastVisualization = true;	
			break;
			}
		if (!pointInScene && !pointsInModelSelected[pointSelectionIndex])
			{
			pointsInModelSelected[pointSelectionIndex] = true;
			pointsInModel[pointSelectionIndex].x = x;
			pointsInModel[pointSelectionIndex].y = y;
			pointsInModel[pointSelectionIndex].z = z;
			cloudsChangedSinceLastVisualization = true;	
			break;
			}
		}
	
	allPointsSelected = true;
	for(int pointSelectionIndex = 0; pointSelectionIndex < NumberOfPoints; pointSelectionIndex++)
		{
		allPointsSelected = allPointsSelected && pointsInSceneSelected[pointSelectionIndex] && pointsInModelSelected[pointSelectionIndex];
		}
	}

void CloudsOverlapper::KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event, void* userdata)
	{
	((CloudsOverlapper*)userdata)->KeyboardButtonCallback(event);
	}

void CloudsOverlapper::KeyboardButtonCallback(const pcl::visualization::KeyboardEvent& event)
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
		for(int pointSelectionIndex = NumberOfPoints-1; pointSelectionIndex >= 0; pointSelectionIndex--)
			{
			if (pointsInModelSelected[pointSelectionIndex] || pointsInSceneSelected[pointSelectionIndex])
				{
				pointsInSceneSelected[pointSelectionIndex] = false;
				pointsInModelSelected[pointSelectionIndex] = false;
				cloudsChangedSinceLastVisualization = true;	
				break;
				}
			}
		allPointsSelected = false;
		}
	else if (command == 23) //Ctrl+W
		{
		SetPosition(modelPose, GetXPosition(modelPose), GetYPosition(modelPose)+translationResolution, GetZPosition(modelPose));
		PRINT_TO_LOG("Model pose", ToString(modelPose));
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 24) //Ctrl+X
		{
		SetPosition(modelPose, GetXPosition(modelPose), GetYPosition(modelPose)-translationResolution, GetZPosition(modelPose));
		PRINT_TO_LOG("Model pose", ToString(modelPose));
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 1) //Ctrl+A
		{
		SetPosition(modelPose, GetXPosition(modelPose)-translationResolution, GetYPosition(modelPose), GetZPosition(modelPose));
		PRINT_TO_LOG("Model pose", ToString(modelPose));
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 4) //Ctrl+D
		{
		SetPosition(modelPose, GetXPosition(modelPose)+translationResolution, GetYPosition(modelPose), GetZPosition(modelPose));
		PRINT_TO_LOG("Model pose", ToString(modelPose));
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 5) //Ctrl+E
		{
		SetPosition(modelPose, GetXPosition(modelPose), GetYPosition(modelPose), GetZPosition(modelPose)-translationResolution);
		PRINT_TO_LOG("Model pose", ToString(modelPose));
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 26) //Ctrl+Z
		{
		SetPosition(modelPose, GetXPosition(modelPose), GetYPosition(modelPose), GetZPosition(modelPose)+translationResolution);
		PRINT_TO_LOG("Model pose", ToString(modelPose));
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 20) //Ctrl+T
		{
		RotateModel(0, rotationResolution, 0);
		PRINT_TO_LOG("Model pose", ToString(modelPose));
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 2) //Ctrl+B
		{
		RotateModel(0, -rotationResolution, 0);
		PRINT_TO_LOG("Model pose", ToString(modelPose));
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 6) //Ctrl+F
		{
		RotateModel(-rotationResolution, 0, 0);
		PRINT_TO_LOG("Model pose", ToString(modelPose));
		cloudsChangedSinceLastVisualization = true;	
		}
	else if (command == 8) //Ctrl+H
		{
		RotateModel(rotationResolution, 0, 0);
		PRINT_TO_LOG("Model pose", ToString(modelPose));
		cloudsChangedSinceLastVisualization = true;	
		}
	else if (command == 25) //Ctrl+Y
		{
		RotateModel(0, 0, -rotationResolution);
		PRINT_TO_LOG("Model pose", ToString(modelPose));
		cloudsChangedSinceLastVisualization = true;
		}
	else if (command == 22) //Ctrl+V
		{
		RotateModel(0, 0, rotationResolution);
		PRINT_TO_LOG("Model pose", ToString(modelPose));
		cloudsChangedSinceLastVisualization = true;	
		}
	else if (command == 15) //Ctrl+O
		{
		translationResolution = translationResolution / 2;
		PRINT_TO_LOG("translation resolution", translationResolution);	
		}
	else if (command == 16) //Ctrl+P
		{
		translationResolution = translationResolution * 2;
		PRINT_TO_LOG("translation resolution", translationResolution);	
		}
	else if (command == 11) //Ctrl+K
		{
		rotationResolution = rotationResolution / 2;
		PRINT_TO_LOG("rotation resolution", rotationResolution);	
		}
	else if (command == 12) //Ctrl+L
		{
		rotationResolution = rotationResolution * 2;
		PRINT_TO_LOG("rotation resolution", rotationResolution);		
		}
	else if (command == 10) //Ctrl+j
		{
		if ( allPointsSelected && !modelPoseComputed)
			{
			PRINT_TO_LOG("Computint", "");
			ComputeModelPose();
			cloudsChangedSinceLastVisualization = true;	
			}
		}	
	}

void CloudsOverlapper::VisualizePointClouds()
	{
	Eigen::Quaternion<float> rotation(GetWOrientation(modelPose), GetXOrientation(modelPose), GetYOrientation(modelPose), GetZOrientation(modelPose) );
	Eigen::Translation<float, 3> translation(GetXPosition(modelPose), GetYPosition(modelPose), GetZPosition(modelPose));
	AffineTransform affineTransform = translation * rotation;

	int numberOfPoints = transformedModelCloud->points.size();
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		transformedModelCloud->points.at(pointIndex) = TransformPoint(originalModelCloud->points.at(pointIndex), affineTransform);
		}

	visualizer->removeAllShapes();
	for(int pointSelectionIndex = 0; pointSelectionIndex < NumberOfPoints; pointSelectionIndex++)
		{
		const cv::Scalar& color = COLORS_LIST.at(pointSelectionIndex);
		if (pointsInSceneSelected[pointSelectionIndex])
			{
			std::string sourcePointName = "source_sphere" + std::to_string(pointSelectionIndex);
			visualizer->addSphere(pointsInScene[pointSelectionIndex], visualizationCircleRadius, color[0], color[1], color[2], sourcePointName);
			}

		if (pointsInModelSelected[pointSelectionIndex])
			{
			std::string sinkPointName = "sink_sphere" + std::to_string(pointSelectionIndex);
			visualizer->addSphere(pointsInModel[pointSelectionIndex], visualizationCircleRadius, color[0], color[1], color[2], sinkPointName);
			}
		}

	visualizer->updatePointCloud< pcl::PointXYZ >(transformedModelCloud, modelCloudColor, "modelCloud");
	}

void CloudsOverlapper::RotateModel(float rollChange, float pitchChange, float yawChange)
	{
	Eigen::Quaternionf currentRotation( GetWOrientation(modelPose), GetXOrientation(modelPose), GetYOrientation(modelPose), GetZOrientation(modelPose) );
	auto eulerRotation = currentRotation.toRotationMatrix().eulerAngles(0, 1, 2);

	float newRoll = eulerRotation(0) + rollChange;
	float newPitch = eulerRotation(1) + pitchChange;
	float newYaw = eulerRotation(2) + yawChange;
	Eigen::Quaternionf newRotation = Eigen::AngleAxisf(newRoll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(newPitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(newYaw, Eigen::Vector3f::UnitZ());

	SetOrientation(modelPose, newRotation.x(), newRotation.y(), newRotation.z(), newRotation.w());
	}

pcl::PointXYZ CloudsOverlapper::TransformPoint(const pcl::PointXYZ& point, const AffineTransform& affineTransform)
	{
	Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
	Eigen::Vector3f eigenTransformedPoint = affineTransform * eigenPoint;
	pcl::PointXYZ transformedPoint;
	transformedPoint.x = eigenTransformedPoint.x();
	transformedPoint.y = eigenTransformedPoint.y();
	transformedPoint.z = eigenTransformedPoint.z();
	return transformedPoint;
	}

void CloudsOverlapper::ComputeModelPose()
	{
	for(int pointSelectionIndex = 1; pointSelectionIndex < NumberOfPoints; pointSelectionIndex++)
		{
		float diffX = pointsInScene[pointSelectionIndex].x - pointsInScene[0].x;
		float diffY = pointsInScene[pointSelectionIndex].y - pointsInScene[0].y;
		float diffZ = pointsInScene[pointSelectionIndex].z - pointsInScene[0].z;
		float distance = std::sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
		pointsInScene[pointSelectionIndex].x = pointsInScene[0].x + 0.1*diffX/distance;
		pointsInScene[pointSelectionIndex].y = pointsInScene[0].y + 0.1*diffY/distance;
		pointsInScene[pointSelectionIndex].z = pointsInScene[0].z + 0.1*diffZ/distance;
		}

	for(int pointSelectionIndex = 1; pointSelectionIndex < NumberOfPoints; pointSelectionIndex++)
		{
		float diffX = pointsInModel[pointSelectionIndex].x - pointsInModel[0].x;
		float diffY = pointsInModel[pointSelectionIndex].y - pointsInModel[0].y;
		float diffZ = pointsInModel[pointSelectionIndex].z - pointsInModel[0].z;
		float distance = std::sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
		pointsInModel[pointSelectionIndex].x = pointsInModel[0].x + 0.1*diffX/distance;
		pointsInModel[pointSelectionIndex].y = pointsInModel[0].y + 0.1*diffY/distance;
		pointsInModel[pointSelectionIndex].z = pointsInModel[0].z + 0.1*diffZ/distance;
		}

	Eigen::MatrixXf sceneCentroid = Eigen::MatrixXf::Zero(3, 1);
	Eigen::MatrixXf modelCentroid = Eigen::MatrixXf::Zero(3, 1);
	for(int pointSelectionIndex = 0; pointSelectionIndex < NumberOfPoints; pointSelectionIndex++)
		{
		sceneCentroid(0,0) += pointsInScene[pointSelectionIndex].x;
		sceneCentroid(1,0) += pointsInScene[pointSelectionIndex].y;
		sceneCentroid(2,0) += pointsInScene[pointSelectionIndex].z;
		modelCentroid(0,0) += pointsInModel[pointSelectionIndex].x;
		modelCentroid(1,0) += pointsInModel[pointSelectionIndex].y;
		modelCentroid(2,0) += pointsInModel[pointSelectionIndex].z;
		}
	sceneCentroid(0,0) /= NumberOfPoints;
	sceneCentroid(1,0) /= NumberOfPoints;
	sceneCentroid(2,0) /= NumberOfPoints;
	modelCentroid(0,0) /= NumberOfPoints;
	modelCentroid(1,0) /= NumberOfPoints;
	modelCentroid(2,0) /= NumberOfPoints;

	Eigen::Matrix3f correspondenceMatrix = Eigen::Matrix3f::Zero();
	for(int pointSelectionIndex = 0; pointSelectionIndex < NumberOfPoints; pointSelectionIndex++)
		{
		Eigen::MatrixXf sceneVector(1, 3);
		Eigen::MatrixXf modelVector(3, 1);
		sceneVector << pointsInScene[pointSelectionIndex].x - sceneCentroid(0,0), 
			pointsInScene[pointSelectionIndex].y - sceneCentroid(1,0), pointsInScene[pointSelectionIndex].z - sceneCentroid(2,0);

		modelVector << pointsInModel[pointSelectionIndex].x - modelCentroid(0,0), 
			pointsInModel[pointSelectionIndex].y - modelCentroid(1,0), pointsInModel[pointSelectionIndex].z - modelCentroid(2,0);
		
		correspondenceMatrix = correspondenceMatrix + modelVector*sceneVector;
		}
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(correspondenceMatrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3f rotation = svd.matrixV() * svd.matrixU().transpose();

	if (rotation.determinant() <0)
		{
		rotation(0,2) = -rotation(0,2);
		rotation(1,2) = -rotation(1,2);
		rotation(2,2) = -rotation(2,2);
		}
	Eigen::Quaternionf newRotation(rotation);
	Eigen::MatrixXf translation = - rotation * modelCentroid + sceneCentroid;

	SetPosition(modelPose, translation(0,0), translation(1,0), translation(2,0));
	SetOrientation( modelPose, newRotation.x(), newRotation.y(), newRotation.z(), newRotation.w() );
	modelPoseComputed = true;
	cloudsChangedSinceLastVisualization = true;
	PRINT_TO_LOG("pose", ToString(modelPose));
	}


}
/** @} */
