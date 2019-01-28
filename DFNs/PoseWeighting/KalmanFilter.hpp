/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POSEWEIGHTING_KALMANFILTER_HPP
#define POSEWEIGHTING_KALMANFILTER_HPP

#include "PoseWeightingInterface.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <Helpers/ParametersListHelper.hpp>

namespace CDFF
{
namespace DFN
{
namespace PoseWeighting
{
    /**
     *  DFN that estimates the pose of an element given different predictions. This DFN contains a Kalman Filter and hence it considers the current inputs to the DFN as well as the previous ones.
     */
    class KalmanFilter : public PoseWeightingInterface
    {
        public:

            KalmanFilter();
            virtual ~KalmanFilter();

            virtual void configure();
            virtual void process();

        private:
            struct KalmanFilterOptionsSet
            {
                int numFrames;          //  Number of frames that is waited before estimating a pose to wait for a more stable input
                double maxDistance;     //  Max distance between sequential poses allowed to correct Kalman filter
            };
            Helpers::ParametersListHelper parametersHelper;
            KalmanFilterOptionsSet parameters;
            static const KalmanFilterOptionsSet DEFAULT_PARAMETERS;

            std::unique_ptr<cv::KalmanFilter> m_kalman_filter;
            std::vector<cv::Point> m_last_centers;
            cv::Point m_last_center;
            int m_count;
    };
}
}
}

#endif // POSEWEIGHTING_KALMANFILTER_HPP

/** @} */
