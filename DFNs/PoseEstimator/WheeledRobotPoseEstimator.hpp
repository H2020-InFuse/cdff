/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POSEESTIMATOR_WHEELEDROBOTPOSEESTIMATOR_HPP
#define POSEESTIMATOR_WHEELEDROBOTPOSEESTIMATOR_HPP

#include "PoseEstimatorInterface.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include <opencv2/core.hpp>
#include <vector>
#include <opencv2/video/tracking.hpp>


namespace CDFF
{
namespace DFN
{
namespace PoseEstimator
{
    /**
     * Extracts the pose of a wheeled robot given the recognized wheels as input
     */
    class WheeledRobotPoseEstimator : public PoseEstimatorInterface
    {
        public:

            WheeledRobotPoseEstimator();
            virtual ~WheeledRobotPoseEstimator();

            virtual void configure();
            virtual void process();

        private:

            struct WheeledRobotPoseEstimatorOptionsSet
            {
                // type of wheeled robot to track (MANA / SHERPA)
                std::string robot;
                // stereo camera baseline
                double baseline;
                // maximum acceptable depth retrieved from stereo (~ 20m)
                double maxStereoDepth;
                // set to 'true' to visualize tracker
                bool visualize;
            };

            struct KalmanFilterOptionsSet
            {
                // number of states to be incl. in Kalman Filtering
                // determines the size of the state matrix modeling the system
                int nStates;
                // number of variables measured
                // determines the size of the measurement vector
                int nMeasurements;
                // number of actions control
                // set as zero when the observed system is not commanded
                int nInputs;
                // rate at which the Kalman Filter operates.
                // must fit the rate at which the measures are taken.
                double dt;
                // backup sensor in case measures are not available (no fusion, backup only).
                // set to 'true' if measures from another sensor are allowed
                bool backupMeasurement;
                // metric tolerance on position jumps
                // above this value, KF is re-initialized.
                double maxJumpInPosition;
            };

            Helpers::ParametersListHelper parametersHelper;
            WheeledRobotPoseEstimatorOptionsSet parameters;
            KalmanFilterOptionsSet KFparameters;
            static const WheeledRobotPoseEstimatorOptionsSet DEFAULT_PARAMETERS;
            static const KalmanFilterOptionsSet DEFAULT_KF_PARAMETERS;

            std::vector<cv::Vec3f> filterRobotWheels();
            std::vector<cv::Point> extractRobotCenterAndAxisProjections(std::vector<cv::Vec3f> wheels);
            cv::Point extractForegroundCentroid(cv::Mat imgWithoutBackground);
            cv::Point outputPose(cv::Vec3d position, std::vector<double> orientation);
            cv::Vec3f get3DCoordinates(cv::Point2d point);
            std::vector<double> extractOrientation(std::vector<cv::Vec3f> robotWheels, std::vector<cv::Point> axisPoints);
            void visualizeResults(std::vector<cv::Vec3f> wheels, std::vector<cv::Point> axisPoints);

            cv::Point m_last_center;

            // KF variables
            cv::KalmanFilter KF;
            cv::Mat measurements;
            asn1SccPose prevEstimatedPose;
            double convergenceIdx = 0;
            double prevConvergenceIdx = 0;
            double prevDeltaCov = 0;
            bool converged = false;
    };
}
}
}

#endif // POSEESTIMATOR_WHEELEDROBOTPOSEESTIMATOR_HPP

/** @} */
