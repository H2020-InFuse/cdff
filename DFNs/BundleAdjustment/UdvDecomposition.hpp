/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef UDVDECOMPOSITION_HPP
#define UDVDECOMPOSITION_HPP

#include "BundleAdjustmentInterface.hpp"
#include <FramesSequence.hpp>
#include <PosesSequence.hpp>
#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace dfn_ci
{
	/**
	 * Implementation of the factorization algorithm by Tomasi and Kanade
	 * 
	 * @param, leftCameraMatrix: the camera matrix of the left camera
	 * @param, rightCameraMatrix: the camera maxtrix of the right camera 
	 */
	class UdvDecomposition : public BundleAdjustmentInterface
	{
		public:

			UdvDecomposition();
			virtual ~UdvDecomposition();

			virtual void configure();
			virtual void process();

		private:

			struct CameraMatrix
			{
				float focalLengthX;
				float focalLengthY;
				float principlePointX;
				float principlePointY;
			};

			struct UdvDecompositionOptionsSet
			{
				CameraMatrix leftCameraMatrix;
				CameraMatrix rightCameraMatrix;
				float baseline;
			};

			Helpers::ParametersListHelper parametersHelper;
			UdvDecompositionOptionsSet parameters;
			static const UdvDecompositionOptionsSet DEFAULT_PARAMETERS;

			struct ImagePoint
			{
				int x;
				int y;
				int image;
				bool explored;
				ImagePoint(int initX, int initY, int initImage, bool initExplored)
					{ x = initX; y = initY; image = initImage; explored = initExplored; }
			};

			int ComputeNumberOfImages(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence); 
			cv::Mat ComputeMeasurementMatrix(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence);
			std::vector<ImagePoint> ComputeChainOfMatchingPoints
				(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence, int numberOfImages, ImagePoint point1, ImagePoint point2);
			void AddChainToMeasurementMatrix(const std::vector<ImagePoint> chain, cv::Mat& measurementMatrix);
			void DecomposeMeasurementMatrix(cv::Mat measurementMatrix, cv::Mat& compatibleRotationMatrix, cv::Mat& compatiblePositionMatrix);
			cv::Mat ComputeRotationMatrix(cv::Mat compatibleRotationMatrix, cv::Mat centroidMatrix);
			cv::Mat ComputeMeasuresCentroid(cv::Mat measurementMatrix);
			void CentreMeasurementMatrix(cv::Mat centroidMatrix, cv::Mat& measurementMatrix);
			void ConvertRotationMatrixToPosesSequence(cv::Mat centroidMatrix, cv::Mat rotationMatrix, PoseWrapper::Poses3DSequence& posesSequence);
	
			bool PointIsNotInVector(BaseTypesWrapper::Point2D point, const std::vector<BaseTypesWrapper::Point2D>& vector);
			void ValidateParameters();
			void ValidateInputs();

			bool ThereAreUnexploredPoints(const std::vector<ImagePoint>& chain, ImagePoint& pointToExplore);
			std::vector<int> ComputeMapsToExplore(int numberOfImages, int imageIndex, int& separationPoint);
			int GetSourceIndex(int numberOfImages, int mapIndex, int imageIndex);
			int GetSinkIndex(int numberOfImages, int mapIndex, int imageIndex);
			bool AllImagesAreRepresented(const std::vector<ImagePoint>& chain, int numberOfImages);
			cv::Mat CameraMatrixToCvMatrix(const CameraMatrix& cameraMatrix);
	};
}

#endif // UDVDECOMPOSITION_HPP

/** @} */
