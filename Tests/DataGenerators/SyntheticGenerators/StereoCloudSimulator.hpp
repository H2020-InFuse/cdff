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

		pcl::PointCloud<pcl::PointXYZ>::Ptr ComputePointCloud();

		static void EulerAnglesToQuaternion(double roll, double pitch, double yaw, double& qx, double& qy, double& qz, double& qw);
		static void QuaternionToEulerAngles(double qx, double qy, double qz, double qw, double& roll, double& pitch, double& yaw);
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

		PoseWrapper::Pose3D AddNoiseToCameraPose(std::normal_distribution<double>& viewPositionErrorSource, std::normal_distribution<double>& viewOrientationErrorSource);
		pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloudWithPatchNoise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::normal_distribution<double>& missingPatchErrorSource);

		pcl::PointXYZ TransformPointFromCameraSystemToCloudSystem(pcl::PointXYZ pointInCameraSystem, PoseWrapper::Pose3D cameraPose);
		bool ComputeCameraLineProjectionOnPointCloud(pcl::PointXYZ pointInCloudSystem, pcl::PointXYZ& projectionPoint);
		void InitializeOriginalCloudWithModel00();
		
    };

}
#endif
/* StereoCloudSimulator.hpp */
/** @} */
