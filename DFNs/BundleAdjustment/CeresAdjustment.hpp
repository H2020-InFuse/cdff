/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef CERESADJUSTMENT_HPP
#define CERESADJUSTMENT_HPP

#include "BundleAdjustmentInterface.hpp"
#include <FramesSequence.hpp>
#include <PosesSequence.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <CorrespondenceMaps2DSequenceToMatConverter.hpp>

namespace dfn_ci
{
	/**
	 * Implementation of the factorization algorithm by Tomasi and Kanade
	 * 
	 * @param, leftCameraMatrix: the camera matrix of the left camera
	 * @param, rightCameraMatrix: the camera maxtrix of the right camera 
	 */
	class CeresAdjustment : public BundleAdjustmentInterface
	{
		public:

			CeresAdjustment();
			virtual ~CeresAdjustment();

			virtual void configure();
			virtual void process();

		private:

			struct CameraMatrix
			{
				float focalLengthX;
				float focalLengthY;
				float principalPointX;
				float principalPointY;
			};

			struct CeresAdjustmentOptionsSet
			{
				CameraMatrix leftCameraMatrix;
				CameraMatrix rightCameraMatrix;
				float baseline;
				double squaredPixelErrorTolerance;
			};

			Helpers::ParametersListHelper parametersHelper;
			CeresAdjustmentOptionsSet parameters;
			static const CeresAdjustmentOptionsSet DEFAULT_PARAMETERS;
			bool initialPoseEstimationIsAvailable;
			bool initialPointEstimationIsAvailable;
			int numberOfElementsInFirstCorrespondenceMap;

			struct StereoImagePointCostFunctor 
				{
				StereoImagePointCostFunctor(cv::Mat leftCameraMatrix, cv::Mat rightCameraMatrix, cv::Mat pointMeasuresMatrix, float baseline);
				template <typename T>
				bool operator()(const T* const leftCameraTransform, const T* const point3d, T* residual) const;

				static ceres::CostFunction* Create(cv::Mat leftCameraMatrix, cv::Mat rightCameraMatrix, cv::Mat pointMeasuresMatrix, float baseline);

				cv::Mat leftCameraMatrix, rightCameraMatrix;
				cv::Mat pointMeasuresMatrix;
				float baseline;
				};
			typedef double Point3d[3];
			typedef double Transform3d[6];

			Converters::CorrespondenceMaps2DSequenceToMatConverter correspondencesSequenceConverter;
			cv::Mat leftCameraMatrix, rightCameraMatrix;

			std::vector<cv::Mat> SolveBundleAdjustment(cv::Mat measurementMatrix, bool& success);
			void ConvertProjectionMatricesListToPosesSequence(std::vector<cv::Mat> projectionMatricesList, PoseWrapper::Poses3DSequence& posesSequence);
			void InitializePoints(std::vector<Point3d>& pointCloud, cv::Mat measurementMatrix);
			void InitializePoses(std::vector<Transform3d>& posesSequence, int numberOfImages);
	
			bool PointIsNotInVector(BaseTypesWrapper::Point2D point, const std::vector<BaseTypesWrapper::Point2D>& vector);
			void ValidateParameters();
			void ValidateInputs();
			void ValidateInitialEstimations(int numberOfCameras);
			cv::Mat CameraMatrixToCvMatrix(const CameraMatrix& cameraMatrix);

	};
}

#endif // CERESADJUSTMENT_HPP

/** @} */
