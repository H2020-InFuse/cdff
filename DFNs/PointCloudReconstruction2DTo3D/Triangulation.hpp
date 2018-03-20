/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file Triangulation.hpp
 * @date 29/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 * @brief This DFN executes the triangulation algorithms for reconstruction of the 3d pose of a point from its position recorded by two cameras. The algorithm is described in "Multiple view geometry in 
 * computer vision" by Richard Hartley, and Andrew Zisserman, and is implemented by the openCV library 
 * 
 * This DFN implementation requires the following parameters:
 * @param firstCameraMatrix, the first (or left) camera matrix in terms of focal length and principle point coordinates;
 * @param secondCameraMatrix, the second (or right) camera matrix in terms of focal length and principle point coordinates;
 *
 * @{
 */

#ifndef TRIANGULATION_HPP
#define TRIANGULATION_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>
#include <CorrespondenceMap2D.hpp>
#include <Pose.hpp>
#include <PointCloud.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdlib.h>
#include <string>
#include <pcl/keypoints/harris_3d.h>
#include <yaml-cpp/yaml.h>
#include <SupportTypes.hpp>
#include "opencv2/calib3d.hpp"
#include <Helpers/ParametersListHelper.hpp>

namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class Triangulation : public PointCloudReconstruction2DTo3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
        	Triangulation();
        	~Triangulation();
        	void process();
        	void configure();

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
		enum CAMERA_TYPE
			{
			SOURCE_CAMERA,
			SINK_CAMERA
			};

		Helpers::ParametersListHelper parametersHelper;

		struct CameraMatrix
			{
			double focalLengthX;
			double focalLengthY;
			cv::Point2d principlePoint;
			};

		struct TriangulationOptionsSet
			{
			CameraMatrix firstCameraMatrix;
			CameraMatrix secondCameraMatrix;
			};

		cv::Mat firstCameraMatrix;
		cv::Mat secondCameraMatrix;

		TriangulationOptionsSet parameters;
		static const TriangulationOptionsSet DEFAULT_PARAMETERS;
		cv::Mat ConvertToMat(CameraMatrix cameraMatrix);

		cv::Mat Triangulate(cv::Mat projectionMatrix, cv::Mat pointsVectorAtPose1, cv::Mat pointsVectorAtPose2);
		cv::Mat ConvertAtPose(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap, CAMERA_TYPE cameraPoseIdentifier);
		PointCloudWrapper::PointCloudConstPtr Convert(cv::Mat pointCloudMatrix);

		void ValidateParameters();
		void ValidateInputs(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap, PoseWrapper::Pose3DConstPtr pose);
    };
}
#endif
/* Triangulation.hpp */
/** @} */
