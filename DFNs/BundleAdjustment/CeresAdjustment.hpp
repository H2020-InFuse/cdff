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

			/*struct ImagePoint
			{
				int x;
				int y;
				int image;
				bool explored;
				ImagePoint(float initX, float initY, int initImage, bool initExplored)
					{ x = initX; y = initY; image = initImage; explored = initExplored; }
			};*/

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

			Converters::CorrespondenceMaps2DSequenceToMatConverter correspondenceMaps2DSequenceToMatConverter;
			cv::Mat leftCameraMatrix, rightCameraMatrix;

			std::vector<cv::Mat> SolveBundleAdjustment(cv::Mat measurementMatrix, bool& success);
			void ConvertProjectionMatricesListToPosesSequence(std::vector<cv::Mat> projectionMatricesList, PoseWrapper::Poses3DSequence& posesSequence);

			/*int ComputeNumberOfImages(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence); 
			cv::Mat ComputeMeasurementMatrix(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence);
			std::vector<ImagePoint> ComputeChainOfMatchingPoints
				(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& correspondenceMapsSequence, int numberOfImages, ImagePoint point1, ImagePoint point2);
			void AddChainToMeasurementMatrix(const std::vector<ImagePoint> chain, cv::Mat& measurementMatrix);*/
	
			bool PointIsNotInVector(BaseTypesWrapper::Point2D point, const std::vector<BaseTypesWrapper::Point2D>& vector);
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

#endif // CERESADJUSTMENT_HPP

/** @} */
