#include "KFHelper.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "PoseEstimatorInterface.hpp"  // to update for 'poses'
#include <opencv2/video/tracking.hpp>

cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
    cv::Mat euler(3,1,CV_64F);

    double m00 = rotationMatrix.at<double>(0,0);
    double m02 = rotationMatrix.at<double>(0,2);
    double m10 = rotationMatrix.at<double>(1,0);
    double m11 = rotationMatrix.at<double>(1,1);
    double m12 = rotationMatrix.at<double>(1,2);
    double m20 = rotationMatrix.at<double>(2,0);
    double m22 = rotationMatrix.at<double>(2,2);

    double bank, attitude, heading;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        bank = 0;
        attitude = CV_PI/2;
        heading = atan2(m02,m22);
    }
    else if (m10 < -0.998) { // singularity at south pole
        bank = 0;
        attitude = -CV_PI/2;
        heading = atan2(m02,m22);
    }
    else
    {
        bank = atan2(-m12,m11);
        attitude = asin(m10);
        heading = atan2(-m20,m00);
    }

    euler.at<double>(0) = bank;
    euler.at<double>(1) = attitude;
    euler.at<double>(2) = heading;

    return euler;
}

cv::Mat euler2rot(const cv::Mat & euler)
{
    cv::Mat rotationMatrix(3,3,CV_64F);

    double bank = euler.at<double>(0);
    double attitude = euler.at<double>(1);
    double heading = euler.at<double>(2);

    // Assuming the angles are in radians.
    double ch = cos(heading);
    double sh = sin(heading);
    double ca = cos(attitude);
    double sa = sin(attitude);
    double cb = cos(bank);
    double sb = sin(bank);

    double m00, m01, m02, m10, m11, m12, m20, m21, m22;

    m00 = ch * ca;
    m01 = sh*sb - ch*sa*cb;
    m02 = ch*sa*sb + sh*cb;
    m10 = sa;
    m11 = ca*cb;
    m12 = -ca*sb;
    m20 = -sh*ca;
    m21 = sh*sa*cb + ch*sb;
    m22 = -sh*sa*sb + ch*cb;

    rotationMatrix.at<double>(0,0) = m00;
    rotationMatrix.at<double>(0,1) = m01;
    rotationMatrix.at<double>(0,2) = m02;
    rotationMatrix.at<double>(1,0) = m10;
    rotationMatrix.at<double>(1,1) = m11;
    rotationMatrix.at<double>(1,2) = m12;
    rotationMatrix.at<double>(2,0) = m20;
    rotationMatrix.at<double>(2,1) = m21;
    rotationMatrix.at<double>(2,2) = m22;

    return rotationMatrix;
}



// ------------------------------------------- KF functions --------------------------------------------
void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{
    KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-3));       // set process noise R (obtained trhough tuning). default : 1e-5
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-2));   // set measurement noise Q (obtained trhough tuning). default : 1e-4
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
    /* DYNAMIC MODEL */ // (state Matrix A)
    //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
    // position
    KF.transitionMatrix.at<double>(0,3) = dt;
    KF.transitionMatrix.at<double>(1,4) = dt;
    KF.transitionMatrix.at<double>(2,5) = dt;
    KF.transitionMatrix.at<double>(3,6) = dt;
    KF.transitionMatrix.at<double>(4,7) = dt;
    KF.transitionMatrix.at<double>(5,8) = dt;
    KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);
    // orientation
    KF.transitionMatrix.at<double>(9,12) = dt;
    KF.transitionMatrix.at<double>(10,13) = dt;
    KF.transitionMatrix.at<double>(11,14) = dt;
    KF.transitionMatrix.at<double>(12,15) = dt;
    KF.transitionMatrix.at<double>(13,16) = dt;
    KF.transitionMatrix.at<double>(14,17) = dt;
    KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);
    /* MEASUREMENT MODEL */ // (matrix H)
    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
    KF.measurementMatrix.at<double>(0,0) = 1;  // x
    KF.measurementMatrix.at<double>(1,1) = 1;  // y
    KF.measurementMatrix.at<double>(2,2) = 1;  // z
    KF.measurementMatrix.at<double>(3,9) = 1;  // roll
    KF.measurementMatrix.at<double>(4,10) = 1; // pitch
    KF.measurementMatrix.at<double>(5,11) = 1; // yaw

}

void fillMeasurements( cv::Mat &measurements,
                       const cv::Mat &translation_measured, const cv::Mat &rotation_measured)
{
    // Convert rotation matrix to euler angles
    cv::Mat measured_eulers(3, 1, CV_64F);
    measured_eulers = rot2euler(rotation_measured);
    // Set measurement to predict
    measurements.at<double>(0) = translation_measured.at<double>(0); // x
    measurements.at<double>(1) = translation_measured.at<double>(1); // y
    measurements.at<double>(2) = translation_measured.at<double>(2); // z
    measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
    measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
    measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
}

void updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement,
                         cv::Mat &translation_estimated, cv::Mat &rotation_estimated )
{
    // First predict, to update the internal statePre variable
    cv::Mat prediction = KF.predict();
    // The "correct" phase that is going to use the predicted value and our measurement
    cv::Mat estimated = KF.correct(measurement);
    // Estimated translation
    translation_estimated.at<double>(0) = estimated.at<double>(0);
    translation_estimated.at<double>(1) = estimated.at<double>(1);
    translation_estimated.at<double>(2) = estimated.at<double>(2);
    // Estimated euler angles
    cv::Mat eulers_estimated(3, 1, CV_64F);
    eulers_estimated.at<double>(0) = estimated.at<double>(9);
    eulers_estimated.at<double>(1) = estimated.at<double>(10);
    eulers_estimated.at<double>(2) = estimated.at<double>(11);
    // Convert estimated quaternion to rotation matrix
    rotation_estimated = euler2rot(eulers_estimated);
}

asn1SccPose estimatePose(cv::KalmanFilter &KF, asn1SccPose &extractedPose, cv::Mat &measurements, bool predictOnly, bool positionOnly)
{
    // Instantiate measured and estimated t & R
    cv::Mat translation_measured(3, 1, CV_64F), translation_estimated(3, 1, CV_64F);
    cv::Mat rotation_measured(3, 3, CV_64F), rotation_estimated(3, 3, CV_64F);
    asn1SccPose estimatedPose;

    // define position 'measurements' as available (H matrix) or NOT
    if (predictOnly) {
        KF.measurementMatrix.at<double>(0, 0) = 0;  // x
        KF.measurementMatrix.at<double>(1, 1) = 0;  // y
        KF.measurementMatrix.at<double>(2, 2) = 0;  // z
        KF.measurementMatrix.at<double>(3, 9) = 0;  // roll
        KF.measurementMatrix.at<double>(4, 10) = 0; // pitch
        KF.measurementMatrix.at<double>(5, 11) = 0; // yaw
    } else {
        if (positionOnly){
            KF.measurementMatrix.at<double>(0, 0) = 1;  // x
            KF.measurementMatrix.at<double>(1, 1) = 1;  // y
            KF.measurementMatrix.at<double>(2, 2) = 1;  // z
            KF.measurementMatrix.at<double>(3, 9) = 0;  // roll
            KF.measurementMatrix.at<double>(4, 10) = 0; // pitch
            KF.measurementMatrix.at<double>(5, 11) = 0; // yaw
        } else {
            KF.measurementMatrix.at<double>(0, 0) = 1;  // x
            KF.measurementMatrix.at<double>(1, 1) = 1;  // y
            KF.measurementMatrix.at<double>(2, 2) = 1;  // z
            KF.measurementMatrix.at<double>(3, 9) = 1;  // roll
            KF.measurementMatrix.at<double>(4, 10) = 1; // pitch
            KF.measurementMatrix.at<double>(5, 11) = 1; // yaw
        }
    }

    // 2 wheels found -> define standard measurement noise
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-2));

    // get 'measured' position
    translation_measured.at<double>(0) = extractedPose.pos.arr[0];
    translation_measured.at<double>(1) = extractedPose.pos.arr[1];
    translation_measured.at<double>(2) = extractedPose.pos.arr[2];

    // get 'measured' orientation
    Eigen::Quaternionf q;
    q.w() = extractedPose.orient.arr[0];
    q.x() = extractedPose.orient.arr[1];
    q.y() = extractedPose.orient.arr[2];
    q.z() = extractedPose.orient.arr[3];
    Eigen::Matrix3f m1 = q.normalized().toRotationMatrix();
    eigen2cv(m1, rotation_measured);

    // fill the measurements vector
    fillMeasurements(measurements, translation_measured, rotation_measured);

    // update the Kalman filter with good measurements
    updateKalmanFilter( KF, measurements, translation_estimated, rotation_estimated);

    estimatedPose.pos.arr[0] = translation_estimated.at<double>(0);
    estimatedPose.pos.arr[1] = translation_estimated.at<double>(1);
    estimatedPose.pos.arr[2] = translation_estimated.at<double>(2);

    // re-convert rot to quaternion
    Eigen::Matrix3f R_est;
    cv2eigen(rotation_estimated, R_est);
    Eigen::Quaternionf q_est(R_est);
    q_est.normalize();

    estimatedPose.orient.arr[0] = q_est.w();
    estimatedPose.orient.arr[1] = q_est.x();
    estimatedPose.orient.arr[2] = q_est.y();
    estimatedPose.orient.arr[3] = q_est.z();

    return estimatedPose;
}

void checkFilterConvergence(cv::KalmanFilter KF, int nStates, int nMeasurements, int nInputs, double dt, double maxJumpInPosition,
                            const asn1SccPose &prevEstimatedPose, const asn1SccPose& estimatedPose, double &prevConvergenceIdx,
                            double &convergenceIdx, double &prevDeltaCov, bool &converged)
{

    // previous to current pose error
    double deltaPositionRMS = sqrt(
            pow((estimatedPose.pos.arr[0] - prevEstimatedPose.pos.arr[0]),2) +
            pow((estimatedPose.pos.arr[1] - prevEstimatedPose.pos.arr[1]),2) +
            pow((estimatedPose.pos.arr[2] - prevEstimatedPose.pos.arr[2]),2)
    );

    // re-init filter if non-linearities occur
    if (deltaPositionRMS > maxJumpInPosition) {
        initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);
        converged = false;
        convergenceIdx = 0;
    }

    // check KF convergence
    auto traceCov = cv::trace(KF.errorCovPre);
    double trace;
    trace = sum(traceCov)[0];
    convergenceIdx = convergenceIdx + trace;
    double deltaCov = convergenceIdx - prevConvergenceIdx;

    if (abs(deltaCov - prevDeltaCov) < 1e-6 && prevEstimatedPose.pos.arr[0] !=0 && prevEstimatedPose.pos.arr[1] !=0 ){
        converged = true;
    } else {
        converged = false;
    }

    // update variables
    prevConvergenceIdx = convergenceIdx;
    prevDeltaCov = deltaCov;
}

bool checkKFConvergence(cv::KalmanFilter KF, double &prevCovInnovTrace, double convergenceThreshold) {
    bool convergence;
    cv::Mat Measure = KF.measurementMatrix;
    cv::Mat Ppred = KF.errorCovPre;
    cv::Mat Mempty = cv::Mat::zeros(Ppred.rows - Measure.rows, Ppred.cols, CV_64F);
    cv::Mat R = KF.processNoiseCov;
    cv::Mat H;
    vconcat(Measure, Mempty, H);
    cv::Mat Htr = H.t();
    cv::Mat CovInnov = R + H + Ppred * Htr;

    auto trCovInnov = cv::trace(CovInnov);
    double traceCovInnov;
    traceCovInnov = sum(trCovInnov)[0];

    if (fabs(traceCovInnov - prevCovInnovTrace) < fabs(convergenceThreshold) ) {
        convergence = true;
    } else {
        convergence = false;
    }

    prevCovInnovTrace = traceCovInnov;
    return convergence;
}

// ------------------------------------------- end KF functions ------------------------------------------