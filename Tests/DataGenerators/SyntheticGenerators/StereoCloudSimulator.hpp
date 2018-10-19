/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file StereoCloudSimulator.hpp
 * @date 11/10/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class tries to simulate a point cloud as it would be viewed by a stereo camera.
 *  
 *
 * @{
 */

#ifndef STEREO_CLOUD_SIMULATOR_HPP
#define STEREO_CLOUD_SIMULATOR_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <random>
#include <pcl/io/ply_io.h>
#include <Pose.hpp>
#include <stdlib.h>
#include <iostream>
#include <fstream>

namespace DataGenerators {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class StereoCloudSimulator
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
        	StereoCloudSimulator(int cloudIndex);
        	~StereoCloudSimulator();

		void SetCamera(PoseWrapper::Pose3D viewPose, double imagePlanDistance, double imagePlaneResolution, double imagePlaneSize);
		void SetDisplacementNoiseModel(double mean, double standardDeviation);
		void SetMissingPatchNoiseModel(double mean, double standardDeviation);
		void SetViewPoseNoiseModel(double positionMean, double positionStandardDeviation, double orientationMean, double orientationStandardDeviation);

		pcl::PointCloud<pcl::PointXYZ>::Ptr ComputePointCloud(PoseWrapper::Pose3D& cameraPose);

		static void EulerAnglesToQuaternion(double roll, double pitch, double yaw, double& qx, double& qy, double& qz, double& qw);
		static void QuaternionToEulerAngles(double qx, double qy, double qz, double qw, double& roll, double& pitch, double& yaw);

		static void CreateCameraFile(int pathIndex, std::string outputFilePath);
	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */	
	private:
		pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud;
		PoseWrapper::Pose3D viewPose;
		double imagePlanDistance;
		double imagePlaneResolution;
		double imagePlaneSize; //This is both width and height

		double displacementErrorMean;
		double displacementErrorStandardDeviation;
		double missingPatchErrorMean;
		double missingPatchErrorStandardDeviation;
		double viewPositionErrorMean;
		double viewPositionErrorStandardDeviation;
		double viewOrientationErrorMean;
		double viewOrientationStandardDeviation;

		std::default_random_engine randomEngine;

		PoseWrapper::Pose3D AddNoiseToCameraPose(std::normal_distribution<double>& viewPositionErrorSource, std::normal_distribution<double>& viewOrientationErrorSource,
			PoseWrapper::Pose3D& viewPose);
		pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloudWithPatchNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::normal_distribution<double>& missingPatchErrorSource);
		pcl::PointCloud<pcl::PointXYZ>::Ptr TransformCloudInCameraSystem(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		pcl::PointXYZ TransformPointFromCameraSystemToCloudSystem(pcl::PointXYZ pointInCameraSystem, PoseWrapper::Pose3D cameraPose);
		pcl::PointXYZ TransformPointFromCloudSystemToCameraSystem(pcl::PointXYZ pointInCloudSystem, PoseWrapper::Pose3D cameraPose);
		bool ComputeCameraLineProjectionOnPointCloud(pcl::PointXYZ pointInCloudSystem, pcl::PointXYZ& projectionPoint);
		pcl::PointXYZ ApplyRotation(pcl::PointXYZ point, double qx, double qy, double qz, double qw);
		void AddFrustrumToPointCloud(PoseWrapper::Pose3D cameraPose, pcl::PointCloud<pcl::PointXYZ>::Ptr stereoCloud);

		void InitializeOriginalCloudWithModel00();
		static void CreateCameraFileModel00(std::string outputFilePath);		
    };

}
#endif
/* StereoCloudSimulator.hpp */
/** @} */
