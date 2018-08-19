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
#include <FramesSequence.hpp>
#include <PosesSequence.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <CorrespondenceMaps2DSequenceToMatConverter.hpp>

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

			struct SvdDecompositionOptionsSet
			{
				CameraMatrix leftCameraMatrix;
				CameraMatrix rightCameraMatrix;
				float baseline;
			};

			Helpers::ParametersListHelper parametersHelper;
			SvdDecompositionOptionsSet parameters;
			static const SvdDecompositionOptionsSet DEFAULT_PARAMETERS;

			typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;
			Converters::CorrespondenceMaps2DSequenceToMatConverter correspondencesSequenceConverter;

			cv::Mat leftCameraMatrix, rightCameraMatrix;
			cv::Mat leftCameraMatrixInverse, rightCameraMatrixInverse;
			cv::Mat leftAbsoluteConicImage,	rightAbsoluteConicImage;

			void DecomposeMeasurementMatrix(cv::Mat measurementMatrix, cv::Mat& compatibleRotationMatrix, cv::Mat& compatiblePositionMatrix);
			cv::Mat ComputeTranslationMatrix(cv::Mat centroidMatrix);
			cv::Mat ComputeMeasuresCentroid(cv::Mat measurementMatrix);
			void CentreMeasurementMatrix(cv::Mat centroidMatrix, cv::Mat& measurementMatrix);
			void ConvertRotationTranslationMatricesToPosesSequence(cv::Mat translationMatrix, cv::Mat rotationMatrix, PoseWrapper::Poses3DSequence& posesSequence);
			cv::Mat ComputeMetricRotationMatrix(cv::Mat rotationMatrix, int poseIndex);

			void ValidateParameters();
			void ValidateInputs();
			cv::Mat CameraMatrixToCvMatrix(const CameraMatrix& cameraMatrix);
	};
}
}
}

#endif // BUNDLEADJUSTMENT_SVDDECOMPOSITION_HPP

/** @} */
