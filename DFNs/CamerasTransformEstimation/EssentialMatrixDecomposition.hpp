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
	 *        Number of points that need to be tested. Each test verifies that 
	 *        a point match is expected according to the candidate essential matrix.
	 *        If half of the tested points pass the test for a candidate essential matrix,
	 *	  the matrix is accepted as valid output. If one and only one matrix is accepted
	 *	  the computation succeed, otherwise a failure is reported.
	 */
	class EssentialMatrixDecomposition : public CamerasTransformEstimationInterface
	{
		public:

			EssentialMatrixDecomposition();
			virtual ~EssentialMatrixDecomposition();

			virtual void configure() override;
			virtual void process() override;

		private:

			//DFN Parameters
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

			//External conversion helpers
			Converters::MatToTransform3DConverter matToTransform3DConverter;

			//Type conversion methods
			cv::Mat ConvertToMat(CameraMatrix cameraMatrix);
			cv::Mat ConvertToMat(MatrixWrapper::Matrix3dConstPtr matrix);
			cv::Mat Convert(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap);

			//Core computation methods
			std::vector<cv::Mat> ComputeTransformMatrix(cv::Mat fundamentalMatrix);
			int FindValidTransform(std::vector<cv::Mat> transformsList, cv::Mat correspondenceMap);
			bool ProjectionMatrixIsValidForTestPoints(cv::Mat projectionMatrix, cv::Mat correspondenceMap);

			//Input Validation methods
			void ValidateParameters();
			void ValidateInputs(cv::Mat fundamentalMatrix, cv::Mat CorrespondenceMap);
	};
}
}
}

#endif // CAMERASTRANSFORMESTIMATION_ESSENTIALMATRIXDECOMPOSITION_HPP

/** @} */
