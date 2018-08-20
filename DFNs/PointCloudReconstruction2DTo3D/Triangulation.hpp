/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDRECONSTRUCTION2DTO3D_TRIANGULATION_HPP
#define POINTCLOUDRECONSTRUCTION2DTO3D_TRIANGULATION_HPP

#include "PointCloudReconstruction2DTo3DInterface.hpp"

#include <CorrespondenceMap2D.hpp>
#include <Pose.hpp>
#include <PointCloud.hpp>
#include <Transform3DToMatConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/calib3d.hpp>

namespace CDFF
{
namespace DFN
{
namespace PointCloudReconstruction2DTo3D
{

	/**
	 * Reconstruction of the 3D position of a point, or several points, based
	 * on its observation by two cameras, using triangulation (algorithm
	 * described in Richard Hartley and Andrew Zisserman, "Multiple View
	 * Geometry in Computer Vision", and provided by OpenCV).
	 *
	 * @param firstCameraMatrix
	 *        first/left camera matrix, in terms of focal length and coordinates
	 *        of the principal point
	 * @param secondCameraMatrix
	 *        second/right camera matrix, in terms of focal length and
	 *        coordinates of the principal point
	 */
	class Triangulation : public PointCloudReconstruction2DTo3DInterface
	{
		public:

			Triangulation();
			virtual ~Triangulation();

			virtual void configure();
			virtual void process();

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
				cv::Point2d principalPoint;
			};

			struct TriangulationOptionsSet
			{
				CameraMatrix firstCameraMatrix;
				CameraMatrix secondCameraMatrix;
				bool outputInvalidPoints;
			};

			cv::Mat firstCameraMatrix;
			cv::Mat secondCameraMatrix;

			TriangulationOptionsSet parameters;
			static const TriangulationOptionsSet DEFAULT_PARAMETERS;
			cv::Mat ConvertToMat(CameraMatrix cameraMatrix);

			Converters::Pose3DToMatConverter pose3DToMat;
			cv::Mat ConvertAtPose(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap, CAMERA_TYPE cameraPoseIdentifier);
			PointCloudWrapper::PointCloudConstPtr Convert(cv::Mat pointCloudMatrix);

			cv::Mat Triangulate(cv::Mat projectionMatrix, cv::Mat pointsVectorAtPose1, cv::Mat pointsVectorAtPose2);

			void ValidateParameters();
			void ValidateInputs(const CorrespondenceMap2DWrapper::CorrespondenceMap2D& matches, const PoseWrapper::Pose3D& pose);
	};
}
}
}

#endif // POINTCLOUDRECONSTRUCTION2DTO3D_TRIANGULATION_HPP

/** @} */
