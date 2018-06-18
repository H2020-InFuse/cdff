/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef SVDDECOMPOSITION_HPP
#define SVDDECOMPOSITION_HPP

#include "BundleAdjustmentInterface.hpp"
#include <FramesSequence.hpp>
#include <PosesSequence.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <CorrespondenceMaps2DSequenceToMatConverter.hpp>

namespace dfn_ci
{
	/**
	 * Implementation of the factorization algorithm by Tomasi and Kanade
	 * 
	 * @param, leftCameraMatrix: the camera matrix of the left camera
	 * @param, rightCameraMatrix: the camera maxtrix of the right camera 
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

			/*struct ImagePoint
			{
				int x;
				int y;
				int image;
				bool explored;
				ImagePoint(float initX, float initY, int initImage, bool initExplored)
					{ x = initX; y = initY; image = initImage; explored = initExplored; }
			};*/
			typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;
			Converters::CorrespondenceMaps2DSequenceToMatConverter correspondenceMaps2DSequenceToMatConverter;

			cv::Mat leftCameraMatrix, rightCameraMatrix;
			cv::Mat leftCameraMatrixInverse, rightCameraMatrixInverse;
			cv::Mat leftAbsoluteConicImage,	rightAbsoluteConicImage;

			/*int ComputeNumberOfImages(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence); 
			cv::Mat ComputeMeasurementMatrix(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence);
			std::vector<ImagePoint> ComputeChainOfMatchingPoints
				(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence, int numberOfImages, ImagePoint point1, ImagePoint point2);
			void AddChainToMeasurementMatrix(const std::vector<ImagePoint> chain, cv::Mat& measurementMatrix);*/
			void DecomposeMeasurementMatrix(cv::Mat measurementMatrix, cv::Mat& compatibleRotationMatrix, cv::Mat& compatiblePositionMatrix);
			cv::Mat ComputeTranslationMatrix(cv::Mat centroidMatrix);
			cv::Mat ComputeMeasuresCentroid(cv::Mat measurementMatrix);
			void CentreMeasurementMatrix(cv::Mat centroidMatrix, cv::Mat& measurementMatrix);
			void ConvertRotationTranslationMatricesToPosesSequence(cv::Mat translationMatrix, cv::Mat rotationMatrix, PoseWrapper::Poses3DSequence& posesSequence);
			cv::Mat ComputeMetricRotationMatrix(cv::Mat rotationMatrix, int poseIndex);
	
			//bool PointIsNotInVector(BaseTypesWrapper::Point2D point, const std::vector<BaseTypesWrapper::Point2D>& vector);
			void ValidateParameters();
			void ValidateInputs();

			cv::Mat CameraMatrixToCvMatrix(const CameraMatrix& cameraMatrix);
			/*bool ThereAreUnexploredPoints(const std::vector<ImagePoint>& chain, int& chainIndexToExplore);
			std::vector<int> ComputeMapsToExplore(int numberOfImages, int imageIndex, int& separationPoint);
			int GetSourceIndex(int numberOfImages, int mapIndex, int imageIndex);
			int GetSinkIndex(int numberOfImages, int mapIndex, int imageIndex);
			bool AllImagesAreRepresented(const std::vector<ImagePoint>& chain, int numberOfImages);
			bool ImageIsNotInChain(const std::vector<ImagePoint>& chain, int imageIndex);*/
	};
}

#endif // SVDDECOMPOSITION_HPP

/** @} */
