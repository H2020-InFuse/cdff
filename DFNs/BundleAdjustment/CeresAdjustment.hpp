/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef BUNDLEADJUSTMENT_CERESADJUSTMENT_HPP
#define BUNDLEADJUSTMENT_CERESADJUSTMENT_HPP

#include "BundleAdjustmentInterface.hpp"
#include <Types/CPP/FramesSequence.hpp>
#include <Types/CPP/PosesSequence.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <Converters/CorrespondenceMaps2DSequenceToMatConverter.hpp>

namespace CDFF
{
namespace DFN
{
namespace BundleAdjustment
{
	/**
	 * Implementation of the factorization algorithm by Tomasi and Kanade
	 *
	 * @param leftCameraMatrix: the camera matrix of the left camera
	 * @param rightCameraMatrix: the camera maxtrix of the right camera
	 */
	class CeresAdjustment : public BundleAdjustmentInterface
	{
		public:

			CeresAdjustment();
			virtual ~CeresAdjustment();

			virtual void configure() override;
			virtual void process() override;

		private:

			//DFN Parameters
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

			//Ceres Functor
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

			//Internal State variables
			bool initialPoseEstimationIsAvailable;
			bool initialPointEstimationIsAvailable;

			//Configuration Parameters conversion
			cv::Mat leftCameraMatrix, rightCameraMatrix;
			cv::Mat CameraMatrixToCvMatrix(const CameraMatrix& cameraMatrix);

			//External conversion helpers
			Converters::CorrespondenceMaps2DSequenceToMatConverter correspondencesSequenceConverter;

			//Internal Type Conversion Methods
			void ConvertProjectionMatricesListToPosesSequence(std::vector<cv::Mat> projectionMatricesList, PoseWrapper::Poses3DSequence& posesSequence);

			//Core Computation Methods
			std::vector<cv::Mat> SolveBundleAdjustment(cv::Mat measurementMatrix, bool& success);
			void InitializePoints(std::vector<Point3d>& pointCloud, cv::Mat measurementMatrix);
			void InitializePoses(std::vector<Transform3d>& posesSequence, int numberOfImages);
			bool PointIsNotInVector(BaseTypesWrapper::Point2D point, const std::vector<BaseTypesWrapper::Point2D>& vector);

			//Validation Methods
			void ValidateParameters();
			void ValidateInputs();
			void ValidateInitialEstimations(int numberOfCameras);

	};
}
}
}

#endif // BUNDLEADJUSTMENT_CERESADJUSTMENT_HPP

/** @} */
