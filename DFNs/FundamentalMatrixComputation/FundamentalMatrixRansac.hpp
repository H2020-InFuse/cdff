/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FUNDAMENTALMATRIXCOMPUTATION_FUNDAMENTALMATRIXRANSAC_HPP
#define FUNDAMENTALMATRIXCOMPUTATION_FUNDAMENTALMATRIXRANSAC_HPP

#include "FundamentalMatrixComputationInterface.hpp"

#include <CorrespondenceMap2D.hpp>
#include <Matrix.hpp>
#include <BaseTypes.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace CDFF
{
namespace DFN
{
namespace FundamentalMatrixComputation
{
	/**
	 * Estimation of the fundamental matrix of a camera pair from a set of 2D
	 * keypoint pairs, using RANSAC (RANdom SAmple Consensus, provided by
	 * OpenCV).
	 *
	 * @param outlierThreshold
	 *        Pixel distance that determines whether an input point (a keypoint
	 *        pair, in our case) is an outlier in the RANSAC model.
	 *        Recommended values: 1, 2, and 3.
	 * @param confidence
	 *        Lowest accepted value for the probability that a RANSAC model
	 *        represents the set of points defined by the set of keypoint pairs.
	 * @param maximumSymmetricEpipolarDistance
	 *	  Lowest accepted symmetric epipolar distance error to accept
	 *	  a correspondence as an inlier.
	 */
	class FundamentalMatrixRansac : public FundamentalMatrixComputationInterface
	{
		public:

			FundamentalMatrixRansac();
			virtual ~FundamentalMatrixRansac();

			virtual void configure();
			virtual void process();

		private:

			struct FundamentalMatrixRansacOptionsSet
			{
				double outlierThreshold; // in pixels
				double confidence;       // probability value (in [0,1])
				double maximumSymmetricEpipolarDistance; //in pixels
			};

			Helpers::ParametersListHelper parametersHelper;
			FundamentalMatrixRansacOptionsSet parameters;
			static const FundamentalMatrixRansacOptionsSet DEFAULT_PARAMETERS;

			cv::Mat ComputeFundamentalMatrix(
				const std::vector<cv::Point2d>& firstImagePointsVector,
				const std::vector<cv::Point2d>& secondImagePointsVector);
			BaseTypesWrapper::Point2DConstPtr ComputeSecondEpipole(
				const std::vector<cv::Point2d>& firstImagePointsVector,
				const std::vector<cv::Point2d>& secondImagePointsVector,
				cv::Mat fundamentalMatrix);
			void ComputeInliers(cv::Mat fundamentalMatrix);
			void Convert(
				CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap,
				std::vector<cv::Point2d>& firstImagePointsVector,
				std::vector<cv::Point2d>& secondImagePointsVector);
			MatrixWrapper::Matrix3dConstPtr Convert(cv::Mat matrix);


			void ValidateParameters();
			void ValidateInputs(
				const std::vector<cv::Point2d>& firstImagePointsVector,
				const std::vector<cv::Point2d>& secondImagePointsVector);
	};
}
}
}

#endif // FUNDAMENTALMATRIXCOMPUTATION_FUNDAMENTALMATRIXRANSAC_HPP

/** @} */
