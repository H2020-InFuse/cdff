/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StereoCloudSimulator.cpp
 * @date 11/10/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the StereoCloudSimulator class.
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
#include "StereoCloudSimulator.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


using namespace PoseWrapper;

namespace DataGenerators
{

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
StereoCloudSimulator::StereoCloudSimulator(int cloudIndex) :
	originalCloud(new pcl::PointCloud<pcl::PointXYZ>)
	{
	SetPosition(viewPose, 0, 0, 0);
	SetOrientation(viewPose, 0, 0, 0, 1);

	imagePlanDistance = 0.001;
	imagePlaneResolution = 1e-6;

	displacementErrorMean = 0;
	displacementErrorStandardDeviation = 0;	
	missingPatchErrorMean = 0;
	missingPatchErrorStandardDeviation = 0;
	viewPositionErrorMean = 0;
	viewPositionErrorStandardDeviation = 0;
	viewOrientationErrorMean = 0;
	viewOrientationStandardDeviation = 0;

	if (cloudIndex == 0)
		{
		InitializeOriginalCloudWithModel00();
		}
	else
		{
		std::cout << "Index out of range" << std::endl;
		abort();
		}
	}

StereoCloudSimulator::~StereoCloudSimulator()
	{

	}

void StereoCloudSimulator::SetCamera(PoseWrapper::Pose3D viewPose, double imagePlanDistance, double imagePlaneResolution, double imagePlaneSize)
	{
	this->viewPose = viewPose;
	this->imagePlanDistance = imagePlanDistance;
	this->imagePlaneResolution = imagePlaneResolution;
	this->imagePlaneSize = imagePlaneSize;
	}

void StereoCloudSimulator::SetDisplacementNoiseModel(double mean, double standardDeviation)
	{
	displacementErrorMean = mean;
	displacementErrorStandardDeviation = standardDeviation;
	}

void StereoCloudSimulator::SetMissingPatchNoiseModel(double mean, double standardDeviation)
	{
	missingPatchErrorMean = mean;
	missingPatchErrorStandardDeviation = standardDeviation;
	}

void StereoCloudSimulator::SetViewPoseNoiseModel(double positionMean, double positionStandardDeviation, double orientationMean, double orientationStandardDeviation)
	{
	viewPositionErrorMean = positionMean;
	viewPositionErrorStandardDeviation = positionStandardDeviation;
	viewOrientationErrorMean = orientationMean;
	viewOrientationStandardDeviation = orientationStandardDeviation;
	}

pcl::PointCloud<pcl::PointXYZ>::Ptr StereoCloudSimulator::ComputePointCloud()
	{
	const double resolution = 1e-3;
	std::normal_distribution<double> displancementErrorSource(displacementErrorMean, displacementErrorStandardDeviation);
	std::normal_distribution<double> missingPatchErrorSource(missingPatchErrorMean, missingPatchErrorStandardDeviation);
	std::normal_distribution<double> viewPositionErrorSource(viewPositionErrorMean, viewPositionErrorStandardDeviation);
	std::normal_distribution<double> viewOrientationErrorSource(viewOrientationErrorMean, viewOrientationStandardDeviation);
	pcl::PointCloud<pcl::PointXYZ>::Ptr stereoCloud(new pcl::PointCloud<pcl::PointXYZ>);

	Pose3D cameraPose = AddNoiseToCameraPose(viewPositionErrorSource, viewOrientationErrorSource);
	int numberOfStepsInEachDirection = imagePlaneSize/(2*imagePlaneResolution);
	pcl::PointXYZ planePoint; // this is the point in the camera system
	planePoint.z = imagePlanDistance;
	for(int horizontalStep = 0; horizontalStep < numberOfStepsInEachDirection; horizontalStep = (horizontalStep > 0) ? -horizontalStep : -horizontalStep + 1)
		{
		planePoint.x = horizontalStep * imagePlaneResolution;
		for(int verticalStep = 0; verticalStep < numberOfStepsInEachDirection; verticalStep = (verticalStep > 0) ? -verticalStep : -verticalStep + 1)
			{
			planePoint.y = verticalStep * imagePlaneResolution;
			//std::cout << "Working on plane point (" << planePoint.x << ", " << planePoint.y << ", " << planePoint.z << ")" << std::endl;

			pcl::PointXYZ transformedPoint = TransformPointFromCameraSystemToCloudSystem(planePoint, cameraPose);

			//std::cout << "Working on transformed plane point (" << transformedPoint.x << ", " << transformedPoint.y << ", " << transformedPoint.z << ")" << std::endl;
			pcl::PointXYZ projection;
			bool projectionExists = ComputeCameraLineProjectionOnPointCloud(transformedPoint, projection);
			projection.x = projection.x + displancementErrorSource(randomEngine);
			projection.y = projection.y + displancementErrorSource(randomEngine);
			projection.z = projection.z + displancementErrorSource(randomEngine);
			if (projectionExists)
				{
				//std::cout << "Projection point is: (" << projection.x << ", " << projection.y << ", " << projection.z << ")" << std::endl;
				stereoCloud->points.push_back(projection);
				}
			}
		}

	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(stereoCloud);
	grid.setLeafSize(resolution, resolution, resolution);
	grid.filter(*stereoCloud);

	return GetCloudWithPatchNoise(stereoCloud, missingPatchErrorSource);
	}

void StereoCloudSimulator::EulerAnglesToQuaternion(double roll, double pitch, double yaw, double& qx, double& qy, double& qz, double& qw)
	{
	double cosYaw = std::cos(yaw/2);
	double sinYaw = std::sin(yaw/2);
	double cosRoll = std::cos(roll/2);
	double sinRoll = std::sin(roll/2);
	double cosPitch = std::cos(pitch/2);
	double sinPitch = std::sin(pitch/2);

	qw = cosYaw * cosRoll * cosPitch + sinYaw * sinRoll * sinPitch;
	qx = cosYaw * sinRoll * cosPitch - sinYaw * cosRoll * sinPitch;
	qy = cosYaw * cosRoll * sinPitch + sinYaw * sinRoll * cosPitch;
	qz = sinYaw * cosRoll * cosPitch - cosYaw * sinRoll * sinPitch;
	}

void StereoCloudSimulator::QuaternionToEulerAngles(double qx, double qy, double qz, double qw, double& roll, double& pitch, double& yaw)
	{
	double sinRollcosPitch = 2.0 * (qw * qx + qy * qz);
	double cosRollcosPitch = 1.0 - 2.0 * (qx * qx + qy * qy);
	roll = std::atan2(sinRollcosPitch, cosRollcosPitch);

	double sinPitch = 2.0 * (qw * qy - qz * qx);
	sinPitch = (sinPitch < -1 ) ? -1 : (sinPitch > 1 ? 1 : sinPitch);
	pitch = std::asin(sinPitch);

	double sinYawCosPitch = 2.0 * (qw * qz + qx * qy);
	double cosYawCosPitch = 1.0 - 2.0 * (qy * qy + qz * qz);  
	yaw = std::atan2(sinYawCosPitch, cosYawCosPitch);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
Pose3D StereoCloudSimulator::AddNoiseToCameraPose(std::normal_distribution<double>& viewPositionErrorSource, std::normal_distribution<double>& viewOrientationErrorSource)
	{
	double x = GetXPosition(viewPose);
	double y = GetYPosition(viewPose);
	double z = GetZPosition(viewPose);
	double qx = GetXOrientation(viewPose);
	double qy = GetYOrientation(viewPose);
	double qz = GetZOrientation(viewPose);
	double qw = GetWOrientation(viewPose);

	double noisyX = x + viewPositionErrorSource(randomEngine);
	double noisyY = y + viewPositionErrorSource(randomEngine);
	double noisyZ = z + viewPositionErrorSource(randomEngine);

	double roll, pitch, yaw;
	QuaternionToEulerAngles(qx, qy, qz, qw, roll, pitch, yaw);

	double noisyRoll = roll + viewOrientationErrorSource(randomEngine);
	double noisyPitch = pitch + viewOrientationErrorSource(randomEngine);
	double noisyYaw = yaw + viewOrientationErrorSource(randomEngine);
	
	double noisyQX, noisyQY, noisyQZ, noisyQW;
	EulerAnglesToQuaternion(noisyRoll, noisyPitch, noisyYaw, noisyQX, noisyQY, noisyQZ, noisyQW);

	Pose3D cameraPose;
	SetPosition(cameraPose, noisyX, noisyY, noisyZ);
	SetOrientation(cameraPose, noisyQX, noisyQY, noisyQZ, noisyQW);
	return cameraPose;
	}

pcl::PointCloud<pcl::PointXYZ>::Ptr StereoCloudSimulator::GetCloudWithPatchNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::normal_distribution<double>& missingPatchErrorSource)
	{
	const double resolution = 1e-3;

	pcl::KdTreeFLANN<pcl::PointXYZ> searchTree;
	searchTree.setInputCloud(cloud);
	std::vector<int> indexList;
	std::vector<float> squaredDistanceList;

	int numberOfPoints = cloud->points.size();
	std::set<int> indexListToRemove;
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		pcl::PointXYZ point = cloud->points.at(pointIndex);
		double noise = missingPatchErrorSource(randomEngine);
		if (noise <= resolution)
			{
			continue;
			}		

		if( searchTree.radiusSearch(point, noise - resolution, indexList, squaredDistanceList) > 0 )
			{
			std::copy( indexList.begin(), indexList.end(), std::inserter( indexListToRemove, indexListToRemove.end() ) );
			}
		}

	pcl::PointCloud<pcl::PointXYZ>::Ptr patchedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		if (indexListToRemove.find(pointIndex) == indexListToRemove.end())
			{
			pcl::PointXYZ point = cloud->points.at(pointIndex);
			patchedCloud->points.push_back(point);
			}
		}
	return patchedCloud;
	}

pcl::PointXYZ StereoCloudSimulator::TransformPointFromCameraSystemToCloudSystem(pcl::PointXYZ pointInCameraSystem, Pose3D cameraPose)
	{
	typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;
	Eigen::Quaternion<float> rotation(GetWRotation(cameraPose), GetXRotation(cameraPose), GetYRotation(cameraPose), GetZRotation(cameraPose));
	Eigen::Translation<float, 3> translation( GetXPosition(cameraPose), GetYPosition(cameraPose), GetZPosition(cameraPose));
	AffineTransform affineTransform = translation * rotation.inverse();

	Eigen::Vector3f eigenPoint(pointInCameraSystem.x, pointInCameraSystem.y, pointInCameraSystem.z);
	Eigen::Vector3f eigenTransformedPoint = affineTransform * eigenPoint;

	pcl::PointXYZ projectionPoint;
	projectionPoint.x = eigenTransformedPoint.x();
	projectionPoint.y = eigenTransformedPoint.y();
	projectionPoint.z = eigenTransformedPoint.z();

	return projectionPoint;
	}

bool StereoCloudSimulator::ComputeCameraLineProjectionOnPointCloud(pcl::PointXYZ pointInCloudSystem, pcl::PointXYZ& projectionPoint)
	{
	const double DistanceLimit = 1e-3;

	double x1 = GetXPosition(viewPose);
	double y1 = GetYPosition(viewPose);
	double z1 = GetZPosition(viewPose);

	double x2 = pointInCloudSystem.x;
	double y2 = pointInCloudSystem.y;
	double z2 = pointInCloudSystem.z;

	double cameraPointDeltaX = x2 - x1;
	double cameraPointDeltaY = y2 - y1;
	double cameraPointDeltaZ = z2 - z1;
	double squaredCameraPointDistance = cameraPointDeltaX*cameraPointDeltaX + cameraPointDeltaY*cameraPointDeltaY + cameraPointDeltaZ*cameraPointDeltaZ;

	int minimumDistanceIndex;
	double minimumDistance;
	double minimumCameraDistance;
	bool minimumDistanceSet = false;
	int numberOfPoints = originalCloud->points.size();
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		pcl::PointXYZ cloudPoint = originalCloud->points.at(pointIndex);
		double x0 = cloudPoint.x;
		double y0 = cloudPoint.y;
		double z0 = cloudPoint.z;
		double scalarProduct = (x1 - x0)*cameraPointDeltaX + (y1 - y0)*cameraPointDeltaY + (z1 - z0) *cameraPointDeltaZ; //dot product of (x1,y1,z1)-(x0,y0,z0) with (x2,y2,z2)-(x1,y1,z1)

		//Parameter t of the closest point to cloudPoint on the line from viewPose to pointInCloudSystem in the parametric equation
		double lineParameter = - scalarProduct/squaredCameraPointDistance; 

		double projX = x1 + (x2 - x1)*lineParameter;
		double projY = y1 + (y2 - y1)*lineParameter;
		double projZ = z1 + (z2 - z1)*lineParameter;

		double deltaX = projX - x0;
		double deltaY = projY - y0;
		double deltaZ = projZ - z0;		
		double distance = std::sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);

		if (distance > DistanceLimit)
			{
			continue;
			}

		double cameraDeltaX = projX - x1;
		double cameraDeltaY = projY - y1;
		double cameraDeltaZ = projZ - z1;
		double cameraDistance = std::sqrt(cameraDeltaX*cameraDeltaX + cameraDeltaY*cameraDeltaY + cameraDeltaZ*cameraDeltaZ);

		if (!minimumDistanceSet  || minimumCameraDistance > cameraDistance + DistanceLimit ||
			(minimumCameraDistance > cameraDistance - DistanceLimit && minimumDistance > distance) )
			{
			minimumDistanceSet = true;
			minimumDistance = distance;
			minimumDistanceIndex = pointIndex;
			minimumCameraDistance = cameraDistance;
			}
		}

	if (!minimumDistanceSet)
		{
		return false;
		}

	//std::cout << "Distance: " << minimumDistance << std::endl;
	projectionPoint = originalCloud->points.at(minimumDistanceIndex);
	return true;
	}

#define NEXT_BUT_LAST_IS_CUBE_SIZE(x) x = (x + CubeResolution < CubeSize) ? (x + CubeResolution) : ( x < CubeSize ? CubeSize : CubeSize + CubeResolution)

void StereoCloudSimulator::InitializeOriginalCloudWithModel00()
	{
	//A cube
	const double CubeResolution = 1e-3;
	const double CubeSize = 0.1;
	std::cout << "The original point cloud is a cube of size " << CubeSize << " and resolution " << CubeResolution << std::endl;
	std::cout << "Cube construction started" << std::endl;
	
	for(double a = 0; a < CubeSize + CubeResolution/2; a = (a < CubeSize) ? CubeSize : CubeSize + CubeResolution) //a will just be 0 or 1
		{
		for(double b = 0; b < CubeSize + CubeResolution/2; NEXT_BUT_LAST_IS_CUBE_SIZE(b) )
			{
			for(double c = 0; c < CubeSize + CubeResolution/2; NEXT_BUT_LAST_IS_CUBE_SIZE(c) )
				{
				pcl::PointXYZ yzSidePoint(a, b, c);
				pcl::PointXYZ xzSidePoint(b, a, c);
				pcl::PointXYZ xySidePoint(b, c, a);

				originalCloud->points.push_back(yzSidePoint);
				originalCloud->points.push_back(xzSidePoint);
				originalCloud->points.push_back(xySidePoint);
				}
			}
		} 
	std::cout << "Cube construction is complete" << std::endl;
	//pcl::PLYWriter writer;
	//writer.write("/Agridrive1/DLR/Synthetic/Cube.ply", *originalCloud, true);
	}
}
/** @} */
