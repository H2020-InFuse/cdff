/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef BUNDLEADJUSTMENT_SVDDECOMPOSITION_HPP
#define BUNDLEADJUSTMENT_SVDDECOMPOSITION_HPP

#include "BundleAdjustmentInterface.hpp"
#include <Types/CPP/FramesSequence.hpp>
#include <Types/CPP/PosesSequence.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
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
	class SvdDecomposition : public BundleAdjustmentInterface
	{
		public:

			SvdDecomposition();
			virtual ~SvdDecomposition();

			virtual void configure() override;
			virtual void process() override;

		private:
			typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;

			//DFN Parameters
			struct CameraMatrix
			{
				float focalLengthX;
				float focalLengthY;
				float principalPointX;
				float principalPointY;
			};

			struct SvdDecompositionOptionsSet
			{
				CameraMatrix leftCameraMatrix;
				CameraMatrix rightCameraMatrix;
				float baseline;
			};

			Helpers::ParametersListHelper parametersHelper;
			SvdDecompositionOptionsSet parameters;
			static const SvdDecompositionOptionsSet DEFAULT_PARAMETERS;

			//External conversion helpers
			Converters::CorrespondenceMaps2DSequenceToMatConverter correspondencesSequenceConverter;

			//Configuration Parameters conversion
			cv::Mat leftCameraMatrix, rightCameraMatrix;
			cv::Mat leftCameraMatrixInverse, rightCameraMatrixInverse;
			cv::Mat leftAbsoluteConicImage,	rightAbsoluteConicImage;
			cv::Mat CameraMatrixToCvMatrix(const CameraMatrix& cameraMatrix);

			//Internal Type Conversion Methods
			void ConvertRotationTranslationMatricesToPosesSequence(cv::Mat translationMatrix, cv::Mat rotationMatrix, PoseWrapper::Poses3DSequence& posesSequence);

			//Core Computation Methods
			void DecomposeMeasurementMatrix(cv::Mat measurementMatrix, cv::Mat& compatibleRotationMatrix, cv::Mat& compatiblePositionMatrix);
			cv::Mat ComputeTranslationMatrix(cv::Mat centroidMatrix);
			cv::Mat ComputeMeasuresCentroid(cv::Mat measurementMatrix);
			void CentreMeasurementMatrix(cv::Mat centroidMatrix, cv::Mat& measurementMatrix);
			cv::Mat ComputeMetricRotationMatrix(cv::Mat rotationMatrix, int poseIndex);

			//Validation Methods
			void ValidateParameters();
			void ValidateInputs();
	};
}
}
}

#endif // BUNDLEADJUSTMENT_SVDDECOMPOSITION_HPP

/** @} */
