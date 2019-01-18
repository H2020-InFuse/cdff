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

            void extractRobotPoints(std::vector<cv::Vec3f> wheels);

            cv::Point extractForegroundCentroid(cv::Mat imgWithoutBackground);

            cv::Point outputPose(cv::Vec3d position, std::vector<double> orientation);

            cv::Vec3f get3DCoordinates(cv::Point point);
            double getDisparityAroundPoint(cv::Point point, const cv::Mat & disparity);

            std::vector<double> extractOrientation();

            void visualizeResults();

            double m_last_left_wheel_x;

            // KF variables
            cv::KalmanFilter KF;
            cv::Mat measurements;
            asn1SccPose prevEstimatedPose;
            double convergenceIdx, prevConvergenceIdx, prevDeltaCov, prevCovInnovTrace;
            bool converged;

            struct WheelPoints {
                cv::Vec3d center;
                cv::Vec3d up;
                cv::Vec3d down;
            };

            struct RobotPoints {
                WheelPoints left_wheel;
                WheelPoints right_wheel;
                cv::Point robot_center_px;
                cv::Vec3d robot_center_3D;
            };

            RobotPoints m_robot_points;
};
}
}
}

#endif // POSEESTIMATOR_WHEELEDROBOTPOSEESTIMATOR_HPP

/** @} */
