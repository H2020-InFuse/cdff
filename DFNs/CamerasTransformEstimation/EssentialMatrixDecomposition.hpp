/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef CAMERASTRANSFORMESTIMATION_ESSENTIALMATRIXDECOMPOSITION_HPP
#define CAMERASTRANSFORMESTIMATION_ESSENTIALMATRIXDECOMPOSITION_HPP

#include "CamerasTransformEstimationInterface.hpp"

#include <Types/CPP/CorrespondenceMap2D.hpp>
#include <Types/CPP/Matrix.hpp>
#include <Converters/MatToTransform3DConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace CDFF
{
namespace DFN
{
namespace CamerasTransformEstimation
{

	/**
	 * Estimation of the geometric transformation between two cameras by
	 * decomposition of the essential matrix.
	 *
	 * Processing steps:
	 *
	 * (i)   Use the camera parameters and the fundamental matrix of the camera
	 *       pair to compute the essential matrix
	 * (ii)  Decompose the essential matrix using an algorithm provided by
	 *       OpenCV and described in Richard Hartley and Andrew Zisserman,
	 *       "Multiple View Geometry in Computer Vision".
	 * (iii) The previous step yields four candidate transformations between
	 *       the two camera frames, but we can use matching keypoints in a pair
	 *       of images captured by the cameras to try and rule out three of
	 *       them.
	 *       If less than three candidate transformations can be ruled out,
	 *       failure is reported and an invalid transformation is returned.
	 *
	 * @param firstCameraMatrix
	 *        Internal parameters of the first/left camera:
	 *        focal lengths and principal points
	 * @param secondCameraMatrix
	 *        Internal parameters of the second/right camera:
	 *        focal lengths and principal points
	 * @param numberOfTestPoints
	 *        Number of points that need to be tested from the 2d features
	 *        matches, half of this number represents the minimum number of
	 *        points whose 2d matching agrees with a transform in order to
	 *        consider that transform as valid (???)
	 */
	class EssentialMatrixDecomposition : public CamerasTransformEstimationInterface
	{
		public:

			EssentialMatrixDecomposition();
			virtual ~EssentialMatrixDecomposition();

			virtual void configure();
			virtual void process();

		private:

			Helpers::ParametersListHelper parametersHelper;

			struct CameraMatrix
			{
				double focalLengthX;
				double focalLengthY;
				cv::Point2d principlePoint;
			};

			struct EssentialMatrixDecompositionOptionsSet
			{
				int numberOfTestPoints;
				CameraMatrix firstCameraMatrix;
				CameraMatrix secondCameraMatrix;
			};

			cv::Mat firstCameraMatrix;
			cv::Mat secondCameraMatrix;

			EssentialMatrixDecompositionOptionsSet parameters;
			static const EssentialMatrixDecompositionOptionsSet DEFAULT_PARAMETERS;
			cv::Mat ConvertToMat(CameraMatrix cameraMatrix);
			cv::Mat ConvertToMat(MatrixWrapper::Matrix3dConstPtr matrix);
			cv::Mat Convert(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap);

			Converters::MatToTransform3DConverter matToTransform3DConverter;

			std::vector<cv::Mat> ComputeTransformMatrix(cv::Mat fundamentalMatrix);
			int FindValidTransform(std::vector<cv::Mat> transformsList, cv::Mat correspondenceMap);
			bool ProjectionMatrixIsValidForTestPoints(cv::Mat projectionMatrix, cv::Mat correspondenceMap);

			void ValidateParameters();
			void ValidateInputs(cv::Mat fundamentalMatrix, cv::Mat CorrespondenceMap);
	};
}
}
}

#endif // CAMERASTRANSFORMESTIMATION_ESSENTIALMATRIXDECOMPOSITION_HPP

/** @} */
