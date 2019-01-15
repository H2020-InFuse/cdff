/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POSEESTIMATOR_PRIMITIVESPOSEESTIMATOR_HPP
#define POSEESTIMATOR_PRIMITIVESPOSEESTIMATOR_HPP

#include "PoseEstimatorInterface.hpp"

#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/video/tracking.hpp>
#include <Eigen/Dense>

namespace CDFF
{
namespace DFN
{
namespace PoseEstimator
{
    /**
	 * Estimate the pose of a robot element given the primitives found in the image
	 *
	 */
    class PrimitivesPoseEstimator : public PoseEstimatorInterface
    {
        public:

            PrimitivesPoseEstimator();
            virtual ~PrimitivesPoseEstimator();

            virtual void configure();
            virtual void process();

        private:

            struct PrimitivesPoseEstimatorOptionsSet
            {
                int numFrames; //Number of frames that is waited before estimating a pose to wait for a more stable input
                double maxDistance; //Max distance between positions allowed to update the Kalman Filter
            };

            Helpers::ParametersListHelper parametersHelper;
            PrimitivesPoseEstimatorOptionsSet parameters;
            static const PrimitivesPoseEstimatorOptionsSet DEFAULT_PARAMETERS;

            void SortPrimitives();
            void InitializeKalman(cv::Point measPt);
            cv::Point PredictPosition();
            Eigen::Quaterniond PredictOrientation(const cv::Mat& inputImage, cv::Point point, const cv::Mat& inputDepth);

            std::vector<std::vector<double>> m_primitives;
            std::unique_ptr<cv::KalmanFilter> m_kalman_filter;

            cv::Point m_last_point;
            int m_count;

    };
}
}
}

#endif // POSEESTIMATOR_PRIMITIVESPOSEESTIMATOR_HPP

/** @} */
