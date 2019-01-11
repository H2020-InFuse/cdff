/**
 * @addtogroup DFNs
 * @{
 */

#ifndef WEIGHTINGEXPERT_POSEWEIGHTING_HPP
#define WEIGHTINGEXPERT_POSEWEIGHTING_HPP

#include "WeightingExpertInterface.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <Helpers/ParametersListHelper.hpp>

namespace CDFF
{
namespace DFN
{
namespace WeightingExpert
{
    /**
     * DFN implementation that estimates the pose of an element given different predictions
     */
    class PoseWeighting : public WeightingExpertInterface
    {
        public:

            PoseWeighting();
            virtual ~PoseWeighting();

            virtual void configure();
            virtual void process();

        private:
            struct PoseWeightingOptionsSet
            {
                int numFrames;          //  Number of frames that is waited before estimating a pose to wait for a more stable input
                double maxDistance;     //  Max distance between sequential poses allowed to correct Kalman filter
            };
            Helpers::ParametersListHelper parametersHelper;
            PoseWeightingOptionsSet parameters;
            static const PoseWeightingOptionsSet DEFAULT_PARAMETERS;

            std::unique_ptr<cv::KalmanFilter> m_kalman_filter;
            std::vector<cv::Point> m_last_centers;
            cv::Point m_last_center;
            int m_count;
    };
}
}
}

#endif // WEIGHTINGEXPERT_POSEWEIGHTING_HPP

/** @} */
