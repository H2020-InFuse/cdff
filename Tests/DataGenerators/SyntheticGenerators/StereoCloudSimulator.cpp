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

pcl::PointCloud<pcl::PointXYZ>::Ptr StereoCloudSimulator::ComputePointCloud(PoseWrapper::Pose3D& cameraPose)
	{
	const double resolution = 1e-3;
	std::normal_distribution<double> displancementErrorSource(displacementErrorMean, displacementErrorStandardDeviation);
	std::normal_distribution<double> missingPatchErrorSource(missingPatchErrorMean, missingPatchErrorStandardDeviation);
	std::normal_distribution<double> viewPositionErrorSource(viewPositionErrorMean, viewPositionErrorStandardDeviation);
	std::normal_distribution<double> viewOrientationErrorSource(viewOrientationErrorMean, viewOrientationStandardDeviation);
	pcl::PointCloud<pcl::PointXYZ>::Ptr stereoCloud(new pcl::PointCloud<pcl::PointXYZ>);

	int numberOfStepsInEachDirection = imagePlaneSize/(2*imagePlaneResolution);
	cameraPose = AddNoiseToCameraPose(viewPositionErrorSource, viewOrientationErrorSource, viewPose);

	pcl::PointXYZ planePoint; // this is the point in the camera system
	planePoint.z = imagePlanDistance;
	for(int horizontalStep = 0; horizontalStep < numberOfStepsInEachDirection; horizontalStep = (horizontalStep > 0) ? -horizontalStep : -horizontalStep + 1)
		{
		planePoint.x = horizontalStep * imagePlaneResolution;
		for(int verticalStep = 0; verticalStep < numberOfStepsInEachDirection; verticalStep = (verticalStep > 0) ? -verticalStep : -verticalStep + 1)
			{
			planePoint.y = verticalStep * imagePlaneResolution;
			pcl::PointXYZ transformedPoint = TransformPointFromCameraSystemToCloudSystem(planePoint, viewPose);

			pcl::PointXYZ projection;
			bool projectionExists = ComputeCameraLineProjectionOnPointCloud(transformedPoint, projection);
			projection.x = projection.x + displancementErrorSource(randomEngine);
			projection.y = projection.y + displancementErrorSource(randomEngine);
			projection.z = projection.z + displancementErrorSource(randomEngine);
			if (projectionExists)
				{
				stereoCloud->points.push_back(projection);
				}
			}
		}

	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(stereoCloud);
	grid.setLeafSize(resolution, resolution, resolution);
	grid.filter(*stereoCloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr patchNoiseCloud = GetCloudWithPatchNoise(stereoCloud, missingPatchErrorSource);
	return TransformCloudInCameraSystem(patchNoiseCloud);
	}

void StereoCloudSimulator::EulerAnglesToQuaternion(double roll, double pitch, double yaw, double& qx, double& qy, double& qz, double& qw)
	{
	Eigen::Quaternion<double> quaternion;
	quaternion = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

	qx = quaternion.x();
	qy = quaternion.y();
	qz = quaternion.z();
	qw = quaternion.w();
	}

void StereoCloudSimulator::QuaternionToEulerAngles(double qx, double qy, double qz, double qw, double& roll, double& pitch, double& yaw)
	{
	Eigen::Quaternion<double> quaternion(qw, qx, qy, qz);
	Eigen::Matrix<double,3,1> eulerAngles = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

	roll = eulerAngles(0,0);
	pitch = eulerAngles(1,0);
	yaw = eulerAngles(2,0);
	}

void StereoCloudSimulator::CreateCameraFile(int pathIndex, std::string outputFilePath)
	{
	if (pathIndex == 0)
		{
		CreateCameraFileModel00(outputFilePath);
		}
	else
		{
		std::cout << "Index out of range" << std::endl;
		abort();
		}
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
Pose3D StereoCloudSimulator::AddNoiseToCameraPose(std::normal_distribution<double>& viewPositionErrorSource, std::normal_distribution<double>& viewOrientationErrorSource, 		
	PoseWrapper::Pose3D& viewPose)
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
	std::cout << "Noisy camera (" << noisyX <<", " << noisyY <<", " << noisyZ <<") (" << noisyQX << ", " << noisyQY << ", " << noisyQZ << ", " << noisyQW << ")" << std::endl;  
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

pcl::PointCloud<pcl::PointXYZ>::Ptr StereoCloudSimulator::TransformCloudInCameraSystem(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);

	int numberOfPoints = cloud->points.size();
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		pcl::PointXYZ transformedPoint = TransformPointFromCloudSystemToCameraSystem( cloud->points.at(pointIndex), viewPose);
		transformedCloud->points.push_back(transformedPoint);
		}

	return transformedCloud;
	}

pcl::PointXYZ StereoCloudSimulator::TransformPointFromCameraSystemToCloudSystem(pcl::PointXYZ pointInCameraSystem, Pose3D cameraPose)
	{
	typedef Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;
	Eigen::Quaternion<double> rotation(GetWRotation(cameraPose), GetXRotation(cameraPose), GetYRotation(cameraPose), GetZRotation(cameraPose));
	Eigen::Translation<double, 3> translation( GetXPosition(cameraPose), GetYPosition(cameraPose), GetZPosition(cameraPose));
	AffineTransform affineTransform = translation * rotation.inverse();

	Eigen::Vector3d eigenPoint(pointInCameraSystem.x, pointInCameraSystem.y, pointInCameraSystem.z);
	Eigen::Vector3d eigenTransformedPoint = affineTransform * eigenPoint;

	pcl::PointXYZ projectionPoint;
	projectionPoint.x = eigenTransformedPoint.x();
	projectionPoint.y = eigenTransformedPoint.y();
	projectionPoint.z = eigenTransformedPoint.z();

	return projectionPoint;
	}

pcl::PointXYZ StereoCloudSimulator::TransformPointFromCloudSystemToCameraSystem(pcl::PointXYZ pointInCameraSystem, Pose3D cameraPose)
	{
	typedef Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;
	Eigen::Quaternion<double> rotation(GetWRotation(cameraPose), GetXRotation(cameraPose), GetYRotation(cameraPose), GetZRotation(cameraPose));
	Eigen::Translation<double, 3> translation( GetXPosition(cameraPose), GetYPosition(cameraPose), GetZPosition(cameraPose));
	AffineTransform affineTransform = translation * rotation.inverse();

	Eigen::Vector3d eigenPoint(pointInCameraSystem.x, pointInCameraSystem.y, pointInCameraSystem.z);
	Eigen::Vector3d eigenTransformedPoint = affineTransform.inverse() * eigenPoint;

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

	projectionPoint = originalCloud->points.at(minimumDistanceIndex);
	return true;
	}

pcl::PointXYZ StereoCloudSimulator::ApplyRotation(pcl::PointXYZ point, double qx, double qy, double qz, double qw)
	{
	Eigen::Quaternion<float> rotation(qw, qx, qy, qz);

	Eigen::Vector3f eigenPoint(point.x, point.y, point.z);
	Eigen::Vector3f eigenTransformedPoint = rotation * eigenPoint;

	pcl::PointXYZ rotatedPoint;
	rotatedPoint.x = eigenTransformedPoint.x();
	rotatedPoint.y = eigenTransformedPoint.y();
	rotatedPoint.z = eigenTransformedPoint.z();

	return rotatedPoint;
	}

void StereoCloudSimulator::AddFrustrumToPointCloud(Pose3D cameraPose, pcl::PointCloud<pcl::PointXYZ>::Ptr stereoCloud)
	{
	int numberOfStepsInEachDirection = imagePlaneSize/(2*imagePlaneResolution);
	pcl::PointXYZ planePoint; // this is the point in the camera system
	planePoint.z = imagePlanDistance;
	for(int horizontalStep = 0; horizontalStep < numberOfStepsInEachDirection; horizontalStep = (horizontalStep > 0) ? -horizontalStep : -horizontalStep + 1)
		{
		planePoint.x = horizontalStep * imagePlaneResolution;
		for(int verticalStep = 0; verticalStep < numberOfStepsInEachDirection; verticalStep = (verticalStep > 0) ? -verticalStep : -verticalStep + 1)
			{
			planePoint.y = verticalStep * imagePlaneResolution;

			pcl::PointXYZ transformedPoint = TransformPointFromCameraSystemToCloudSystem(planePoint, cameraPose);
			if ( (horizontalStep == 0 && verticalStep == 0) || 
				(horizontalStep == numberOfStepsInEachDirection-1 && verticalStep == numberOfStepsInEachDirection-1) || 
				(horizontalStep == -numberOfStepsInEachDirection+1 && verticalStep == numberOfStepsInEachDirection-1) || 
				(horizontalStep == numberOfStepsInEachDirection-1 && verticalStep == -numberOfStepsInEachDirection+1) || 
				(horizontalStep == -numberOfStepsInEachDirection+1 && verticalStep == -numberOfStepsInEachDirection+1) )
				{
				double dx = transformedPoint.x - GetXPosition(cameraPose);
				double dy = transformedPoint.y - GetYPosition(cameraPose);
				double dz = transformedPoint.z - GetZPosition(cameraPose);
				double lineLength = std::sqrt(dx*dx + dy*dy + dz*dz);				
				for(double t = 0; t <= 3; t+=0.01)
					{
					pcl::PointXYZ linePoint;
					linePoint.x = GetXPosition(cameraPose) +dx*t; 
					linePoint.y = GetYPosition(cameraPose) +dy*t; 
					linePoint.z = GetZPosition(cameraPose) +dz*t; 
					stereoCloud->points.push_back(linePoint);
					}
				}
			stereoCloud->points.push_back(transformedPoint);
			}
		}
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

				pcl::PointXYZ yzSideRotated = ApplyRotation(yzSidePoint, 0.3894183423, 0, 0, 0.921060994);
				pcl::PointXYZ xzSideRotated = ApplyRotation(xzSidePoint, 0.3894183423, 0, 0, 0.921060994);
				pcl::PointXYZ xySideRotated = ApplyRotation(xySidePoint, 0.3894183423, 0, 0, 0.921060994);

				originalCloud->points.push_back(yzSideRotated);
				originalCloud->points.push_back(xzSideRotated);
				originalCloud->points.push_back(xySideRotated);
				}
			}
		} 
	std::cout << "Cube construction is complete" << std::endl;
	//pcl::PLYWriter writer;
	//writer.write("/Agridrive1/DLR/Synthetic/Cube.ply", *originalCloud, true);
	}

void StereoCloudSimulator::CreateCameraFileModel00(std::string outputFilePath)
	{
	//The camera moves along a circle around the cube 00.
	const double cubeSize = 0.1;
	const double resolution = 0.1;
	const double radius = 2;
	std::ofstream file(outputFilePath.c_str());

	const double planeDistance = 1;
	const double planeResolution = 0.001;
	const double planSize = 0.1;
	const double displacementErrorMean = 0;
	const double displacementErrorStandardDeviation = 0;
	const double missingPatchErrorMean = 0;
	const double missingPatchErrorStandardDeviation = 0;
	const double viewPositionErrorMean = 0;
	const double viewPositionErrorStandardDeviation = 0;
	const double viewOrientationErrorMean = 0;
	const double viewOrientationStandardDeviation = 0;
	
	for(double angle = 0; angle < 2*M_PI; angle += resolution)
		{
		double x = radius*std::sin(angle) + cubeSize/2;
		double z = radius*std::cos(angle) + cubeSize/2;
		double y = cubeSize/2;

		double roll = 0;
		double pitch = M_PI - angle;
		double yaw = 0;

		file << x << " ";
		file << y << " ";
		file << z << " ";
		file << std::setprecision(13);
		file << roll << " ";
		file << pitch << " ";
		file << std::setprecision(5);
		file << yaw << " ";
		file << planeDistance << " ";
		file << planeResolution << " ";
		file << planSize << " ";
		file << displacementErrorMean << " ";
		file << displacementErrorStandardDeviation << " ";
		file << missingPatchErrorMean << " ";
		file << missingPatchErrorStandardDeviation << " ";
		file << viewPositionErrorMean << " ";
		file << viewPositionErrorStandardDeviation << " ";
		file << viewOrientationErrorMean << " ";
		file << viewOrientationStandardDeviation << " ";
		file << std::endl;
		}

	file.close();
	}
}
/** @} */
