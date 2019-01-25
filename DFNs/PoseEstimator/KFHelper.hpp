#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "PoseEstimatorInterface.hpp" // to update for 'poses'
#include <opencv2/video/tracking.hpp>


cv::Mat rot2euler(const cv::Mat & rotationMatrix);
cv::Mat euler2rot(const cv::Mat & euler);

void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);

void fillMeasurements(cv::Mat &measurements,
                       const cv::Mat &translation_measured, const cv::Mat &rotation_measured);

void updateKalmanFilter(cv::KalmanFilter &KF, cv::Mat &measurement,
                         cv::Mat &translation_estimated, cv::Mat &rotation_estimated );

asn1SccPose estimatePose(cv::KalmanFilter &KF, asn1SccPose &extractedPose, cv::Mat &measurements, bool predictOnly, bool positionOnly);

void checkFilterConvergence(cv::KalmanFilter KF, int nStates, int nMeasurements, int nInputs, double dt, double maxJumpInPosition,
                            const asn1SccPose &prevEstimatedPose, const asn1SccPose& estimatedPose, double &prevConvergenceIdx,
                            double &convergenceIdx, double &prevDeltaCov, bool &converged);

bool checkKFConvergence(cv::KalmanFilter KF, double &prevCovInnovTrace, double convergenceThreshold);
