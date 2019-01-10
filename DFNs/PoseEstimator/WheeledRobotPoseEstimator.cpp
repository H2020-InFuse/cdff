/**
 * @addtogroup DFNs
 * @{
 */

#include "WheeledRobotPoseEstimator.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/video/tracking.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <Converters/FrameToMatConverter.hpp>


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

asn1SccPose estimatePose(cv::KalmanFilter &KF, asn1SccPose &extractedPose, cv::Mat &measurements, bool predictOnly)
{
    // Instantiate measured and estimated t & R
    cv::Mat translation_measured(3, 1, CV_64F), translation_estimated(3, 1, CV_64F);
    cv::Mat rotation_measured(3, 3, CV_64F), rotation_estimated(3, 3, CV_64F);
    asn1SccPose estimatedPose;

    // define position 'measurements' as available (H matrix) or NOT
    if (predictOnly == false) {
        KF.measurementMatrix.at<double>(0, 0) = 1;  // x
        KF.measurementMatrix.at<double>(1, 1) = 1;  // y
        KF.measurementMatrix.at<double>(2, 2) = 1;  // z
        KF.measurementMatrix.at<double>(3, 9) = 1;  // roll
        KF.measurementMatrix.at<double>(4, 10) = 1; // pitch
        KF.measurementMatrix.at<double>(5, 11) = 1; // yaw
    } else {
        KF.measurementMatrix.at<double>(0, 0) = 0;  // x
        KF.measurementMatrix.at<double>(1, 1) = 0;  // y
        KF.measurementMatrix.at<double>(2, 2) = 0;  // z
        KF.measurementMatrix.at<double>(3, 9) = 0;  // roll
        KF.measurementMatrix.at<double>(4, 10) = 0; // pitch
        KF.measurementMatrix.at<double>(5, 11) = 0; // yaw
    }

    // 2 wheels found -> define standard measurement noise
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-2));

    // get 'measured' position
    translation_measured.at<double>(0) = extractedPose.pos.arr[0];
    translation_measured.at<double>(1) = extractedPose.pos.arr[1];
    translation_measured.at<double>(2) = extractedPose.pos.arr[2];

    // get 'measured' orientation
    Eigen::Quaternionf q;
    q.x() = extractedPose.orient.arr[0];
    q.y() = extractedPose.orient.arr[1];
    q.z() = extractedPose.orient.arr[2];
    q.w() = extractedPose.orient.arr[3];
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

    estimatedPose.orient.arr[0] = q_est.w();
    estimatedPose.orient.arr[1] = q_est.x();
    estimatedPose.orient.arr[2] = q_est.y();
    estimatedPose.orient.arr[3] = q_est.z();

    return estimatedPose;
}

void checkFilterConvergence(cv::KalmanFilter KF, int nStates, int nMeasurements, int nInputs, double dt, double maxJumpInPosition,
                            asn1SccPose &prevEstimatedPose, const asn1SccPose estimatedPose, double &prevConvergenceIdx,
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
    auto traceCov = cv::trace(KF.errorCovPost);
    double trace;
    trace = sum(traceCov)[0];
    convergenceIdx = convergenceIdx + trace;
    float deltaCov = convergenceIdx - prevConvergenceIdx;

    if (abs(deltaCov - prevDeltaCov) < 1e-6){
        converged = true;
    } else {
        converged = false;
    }

    // update variables
    prevConvergenceIdx = convergenceIdx;
    prevDeltaCov = deltaCov;
}
// ------------------------------------------- end KF functions ------------------------------------------

namespace robots
{
    const std::string SHERPA = "SHERPA";
    const std::string MANA = "MANA";
}

namespace CDFF
{
namespace DFN
{
namespace PoseEstimator
{

WheeledRobotPoseEstimator::WheeledRobotPoseEstimator()
{
    m_last_center = cv::Point(-1, -1);
    parametersHelper.AddParameter<std::string>("GeneralParameters", "Robot", parameters.robot, DEFAULT_PARAMETERS.robot);
    parametersHelper.AddParameter<double>("GeneralParameters", "Baseline", parameters.baseline, DEFAULT_PARAMETERS.baseline);
    parametersHelper.AddParameter<double>("GeneralParameters", "MaxStereoDepth", parameters.maxStereoDepth, DEFAULT_PARAMETERS.maxStereoDepth);
    parametersHelper.AddParameter<bool>("GeneralParameters", "Visualize", parameters.visualize, DEFAULT_PARAMETERS.visualize);
    // KF params
    parametersHelper.AddParameter<int>("KFParameters", "nStates", KFparameters.nStates, DEFAULT_KF_PARAMETERS.nStates);
    parametersHelper.AddParameter<int>("KFParameters", "nMeasurements", KFparameters.nMeasurements, DEFAULT_KF_PARAMETERS.nMeasurements);
    parametersHelper.AddParameter<int>("KFParameters", "nInputs", KFparameters.nInputs, DEFAULT_KF_PARAMETERS.nInputs);
    parametersHelper.AddParameter<double>("KFParameters", "dt", KFparameters.dt, DEFAULT_KF_PARAMETERS.dt);
    parametersHelper.AddParameter<bool>("KFParameters", "backupMeasurement", KFparameters.backupMeasurement, DEFAULT_KF_PARAMETERS.backupMeasurement);
    parametersHelper.AddParameter<double>("KFParameters", "maxJumpInPosition", KFparameters.maxJumpInPosition, DEFAULT_KF_PARAMETERS.maxJumpInPosition);

    // KF init
    measurements = cv::Mat::zeros(KFparameters.nMeasurements,1,CV_64F);
    initKalmanFilter(KF, KFparameters.nStates, KFparameters.nMeasurements, KFparameters.nInputs, KFparameters.dt);
    converged = false;
    convergenceIdx = 0;

    prevEstimatedPose.pos.arr[0] = 0;
    prevEstimatedPose.pos.arr[1] = 0;
    prevEstimatedPose.pos.arr[2] = 0;
    prevEstimatedPose.orient.arr[0] = 1;
    prevEstimatedPose.orient.arr[1] = 0;
    prevEstimatedPose.orient.arr[2] = 0;
    prevEstimatedPose.orient.arr[3] = 0;

}

WheeledRobotPoseEstimator::~WheeledRobotPoseEstimator()
{
}

const WheeledRobotPoseEstimator::WheeledRobotPoseEstimatorOptionsSet WheeledRobotPoseEstimator::DEFAULT_PARAMETERS =
{
        /*robot =*/ robots::MANA,
        /*baseline =*/  0.2,
        /*maxStereoDepth =*/ 20.0,
        /*visualize =*/ false,
};

const WheeledRobotPoseEstimator::KalmanFilterOptionsSet WheeledRobotPoseEstimator::DEFAULT_KF_PARAMETERS =
{
        // --- KF params
        /*nStates =*/ 18,
        /*nMeasurements =*/ 6,
        /*nInputs =*/ 0,
        /*dt =*/ 1.0,
        /*backupMeasurement =*/ false,
        /*maxJumpInPosition =*/ 2.0
};

void WheeledRobotPoseEstimator::configure()
{
    if(configurationFilePath.empty() == false)
    {
        parametersHelper.ReadFile(configurationFilePath);
    }
}

void WheeledRobotPoseEstimator::process()
{
    asn1SccPosesSequence_Initialize(&outPoses);
    asn1SccPose estimatedPose;
    asn1SccPose_Initialize(&estimatedPose);

    cv::Mat inputImage = Converters::FrameToMatConverter().Convert(&inImage);
    cv::Mat inputDepth = cv::Mat::zeros(cv::Size(inDepth.data.cols, inDepth.data.rows), CV_32FC1);
    inputDepth.data = inDepth.data.data.arr;

    std::vector<cv::Vec3f> robot_wheels = filterRobotWheels();

    if(robot_wheels.empty()==false)
    {
        std::vector<cv::Point> axis_points = extractRobotCenterAndAxisProjections(robot_wheels);
        if(axis_points.empty()==false)
        {
            // ---------- CASE 1/3 : wheels position available ----------
            visualizeResults(robot_wheels, axis_points);
            if( m_last_center.x != -1)
            {
                outputPose(get3DCoordinates(axis_points[0]), extractOrientation(robot_wheels, axis_points));
                // lower confidence in measurements (higher measurement noise)
                cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-2));
                estimatedPose = estimatePose(KF, outPoses.arr[0], measurements, 0);
            }
            else
            {
                outputPose(get3DCoordinates(axis_points[0]), std::vector<double>());
            }

            m_last_center = axis_points[0];
        }
    }
    else
    {
        cv::Point centroid = extractForegroundCentroid(inputImage);
        visualizeResults(std::vector<cv::Vec3f>(), std::vector<cv::Point>(1, centroid));
        outputPose(get3DCoordinates(centroid), std::vector<double>());
        m_last_center = cv::Point(-1,-1);

        // ---------- CASE 2/3 : foreground centroid available ----------
        if (centroid.x > 0 && centroid.y > 0 && KFparameters.backupMeasurement == true)
        {
            // lower confidence in measurements (higher measurement noise)
            cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));

            // get blob center 3D coords
            asn1SccPose poseFromCentroid;
            asn1SccPose_Initialize(&poseFromCentroid);
            cv::Vec3f centroid3D = get3DCoordinates(centroid);
            poseFromCentroid.pos.arr[0] = centroid3D[0];
            poseFromCentroid.pos.arr[1] = centroid3D[1];
            poseFromCentroid.pos.arr[2] = centroid3D[2];

            estimatedPose = estimatePose(KF, poseFromCentroid, measurements, 0);

        }
        // ---------- CASE 3/3 : no measures available ----------
        else {
            //  if convergence reached, apply KF prediction only
            if (converged == true){
                estimatedPose = estimatePose(KF, prevEstimatedPose, measurements, 1);
            }
        }
    }

    checkFilterConvergence(KF, KFparameters.nStates, KFparameters.nMeasurements, KFparameters.nInputs, KFparameters.dt, KFparameters.maxJumpInPosition,
                           prevEstimatedPose, estimatedPose, prevConvergenceIdx, convergenceIdx, prevDeltaCov, converged);

    if (converged)
    {
        outPoses.arr[0] = estimatedPose;
    }

    prevEstimatedPose = estimatedPose;
}

cv::Point WheeledRobotPoseEstimator::outputPose(cv::Vec3d position, std::vector<double> orientation)
{
    outPoses.nCount = 1;
    outPoses.arr[0].pos.arr[0] = position[0];
    outPoses.arr[0].pos.arr[1] = position[1];
    outPoses.arr[0].pos.arr[2] = position[2];

    if(orientation.size()==4)
    {
        outPoses.arr[0].orient.arr[0] = orientation[0];
        outPoses.arr[0].orient.arr[1] = orientation[1];
        outPoses.arr[0].orient.arr[2] = orientation[2];
        outPoses.arr[0].orient.arr[3] = orientation[3];
    }
    else
    {
        outPoses.arr[0].orient.arr[0] = 1;
        outPoses.arr[0].orient.arr[1] = 0;
        outPoses.arr[0].orient.arr[2] = 0;
        outPoses.arr[0].orient.arr[3] = 0;
    }
}

std::vector<double> WheeledRobotPoseEstimator::extractOrientation(std::vector<cv::Vec3f> robotWheels, std::vector<cv::Point> axisPoints)
{
    // retrieve wheel coordinates
    cv::Point wheel_center_1 = cv::Point(robotWheels[0][0],robotWheels[0][1]);
    cv::Point wheel_center_2 = cv::Point(robotWheels[1][0],robotWheels[1][1]);
    cv::Vec3f coord3Dwheel1 = get3DCoordinates(wheel_center_1);
    cv::Vec3f coord3Dwheel2 = get3DCoordinates(wheel_center_2);

    // get rover direction vector w.r.t xUnitVector in image frame (x right, y down, z in)
    cv::Vec3f xUnitVec = cv::Vec3f(1,0,0);
    xUnitVec = cv::normalize(xUnitVec);
    cv::Vec3f rover_heading = coord3Dwheel2 - coord3Dwheel1;
    cv::Vec3f rover_heading_normalized = cv::normalize(rover_heading);

    Eigen::Quaternionf qx;
    cv::Vec3f a = xUnitVec.cross(rover_heading_normalized);
    qx.x() = a[0];
    qx.y() = a[1];
    qx.z() = a[2];
    qx.w() = xUnitVec.dot(rover_heading_normalized);

    cv::Vec3f zUnitVec = cv::Vec3f(0,0,1);
    cv::Vec3f coord3Dwheel2UP_left = get3DCoordinates(axisPoints[1]);
    cv::Vec3f coord3Dwheel2DOWN_left = get3DCoordinates(axisPoints[3]);
    cv::Vec3f up_heading = coord3Dwheel2UP_left - coord3Dwheel2DOWN_left;
    cv::Vec3f up_heading_normalized = cv::normalize(up_heading);

    double angle_around_x = acos(zUnitVec.dot(up_heading_normalized));

    Eigen::Quaternionf rotation_around_x = Eigen::AngleAxisf(angle_around_x, Eigen::Vector3f::UnitX())
                                           * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
                                           * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = qx * rotation_around_x;
    if ((m_last_center.x - axisPoints[0].x) > 0)
    {
        Eigen::Quaternionf q_180_z = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
                                     * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
                                     * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
        q = q * q_180_z;
    }

    q.normalize();

    std::vector<double> orientation;
    orientation.push_back(q.w());
    orientation.push_back(q.x());
    orientation.push_back(q.y());
    orientation.push_back(q.z());
    return orientation;
}

cv::Point WheeledRobotPoseEstimator::extractForegroundCentroid(cv::Mat imgWithoutBackground)
{
    cv::Mat thr;
    threshold( imgWithoutBackground,thr,100,255,cv::THRESH_BINARY );
    cv::Moments m = moments(thr,true);
    cv::Point centroid(m.m10/m.m00, m.m01/m.m00);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(imgWithoutBackground, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    if(contours.empty()==false)
    {
        cv::Rect boundRect = cv::boundingRect( contours[0] );

        if( boundRect.height > 100 ) //If contour is not just a shadow or an outlier
        {
            cv::Point output_point;
            output_point.y = centroid.y;
            double max_diff=0, correction=0;
            if(parameters.robot==robots::SHERPA)
            {
                max_diff = boundRect.width/2;
                correction = boundRect.width/8;
            }
            else if(parameters.robot==robots::MANA)
            {
                max_diff = 50;
                correction = boundRect.width/4;
            }

            if( (boundRect.x-centroid.x) > max_diff )
            {
                output_point.x = centroid.x+correction;
            }
            else if ( (boundRect.x-centroid.x) < max_diff )
            {
                output_point.x = centroid.x-correction;
            }
            else
            {
                output_point = centroid;
            }

            return output_point;
        }

        return cv::Point(-1,-1);
    }

    return centroid;
}


std::vector<cv::Point> WheeledRobotPoseEstimator::extractRobotCenterAndAxisProjections(std::vector<cv::Vec3f> wheels)
{
    std::vector<cv::Point> origin_and_axis_points;
    if(wheels.empty() == false)
    {
        cv::Point wheel_center_1 = cv::Point(wheels[0][0], wheels[0][1]);
        cv::Point wheel_center_2 = cv::Point(wheels[1][0], wheels[1][1]);

        double d = cv::norm(wheel_center_1 - wheel_center_2);
        double h = d/3;
        if(parameters.robot == robots::SHERPA)
        {
            cv::norm(wheel_center_1 - wheel_center_2) / 2;
        }

        double alpha = asin((wheel_center_1.y - wheel_center_2.y) / d);
        double beta = M_PI / 2 - alpha;

        cv::Point origin;
        origin.x = wheel_center_1.x + (wheel_center_2.x - wheel_center_1.x) / 2 - cos(beta) * d / 2;
        origin.y = wheel_center_1.y + (wheel_center_2.y - wheel_center_1.y) / 2 - sin(beta) * d / 2;

        cv::Point top_left, top_right, bottom_left, bottom_right;
        top_left.x = wheel_center_1.x - cos(beta) * h;
        top_left.y = wheel_center_1.y - sin(beta) * h;
        top_right.x = wheel_center_2.x - cos(beta) * h;
        top_right.y = wheel_center_2.y - sin(beta) * h;

        bottom_left.x = wheel_center_1.x + cos(beta) * h;
        bottom_left.y = wheel_center_1.y + sin(beta) * h;
        bottom_right.x = wheel_center_2.x + cos(beta) * h;
        bottom_right.y = wheel_center_2.y + sin(beta) * h;

        origin_and_axis_points.push_back(origin);
        origin_and_axis_points.push_back(top_left);
        origin_and_axis_points.push_back(top_right);
        origin_and_axis_points.push_back(bottom_left);
        origin_and_axis_points.push_back(bottom_right);

        double axis_length = h/2;
        if( m_last_center.x != -1)
        {
            cv::Point end_x;
            end_x.x = origin.x + cos(alpha)*axis_length;
            end_x.y = origin.y + sin(-alpha)*axis_length;

            if( (m_last_center.x - origin.x) > 0)
            {
                end_x.x = origin.x - cos(alpha)*axis_length;
                end_x.y = origin.y - sin(-alpha)*axis_length;
            }

            cv::Point end_z;
            end_z.x = origin.x  - cos(beta)*axis_length;
            end_z.y = origin.y - sin(beta)*axis_length;

            origin_and_axis_points.push_back(end_x);
            origin_and_axis_points.push_back(end_z);
        }
    }
    return origin_and_axis_points;
}

std::vector<cv::Vec3f> WheeledRobotPoseEstimator::filterRobotWheels()
{
    //Sort the circles
    std::vector<cv::Vec3f> circles;
    for( int index = 0; index < inPrimitives.nCount; index ++ )
    {
        circles.push_back(cv::Vec3f(inPrimitives.arr[index].arr[0], inPrimitives.arr[index].arr[1], inPrimitives.arr[index].arr[2]));
    }

    std::sort(circles.begin(), circles.end(), [](cv::Vec3f circle1, cv::Vec3f circle2) {
        return circle1[1] > circle2[1];
    });

    std::vector<cv::Vec3f> wheels_position;
    if(circles.size() >= 2)
    {
        if(parameters.robot==robots::SHERPA)
        {
            if (abs(circles[0][1] - circles[1][1]) <= circles[0][2])
            {
                if (circles[0][0] < circles[1][0])
                {
                    wheels_position.push_back(cv::Vec3f(circles[0][0], circles[0][1], circles[0][2]));
                    wheels_position.push_back(cv::Vec3f(circles[1][0], circles[1][1], circles[1][2]));
                }
                else
                {
                    wheels_position.push_back(cv::Vec3f(circles[1][0], circles[1][1], circles[1][2]));
                    wheels_position.push_back(cv::Vec3f(circles[0][0], circles[0][1], circles[0][2]));
                }
            }
        }
        else if(parameters.robot==robots::MANA)
        {
            if( abs(circles[0][1] - circles[1][1]) < circles[0][2] //No circles on top of each other (vertical)
                && abs(circles[0][0] - circles[1][0]) > circles[0][2] && abs(circles[0][0] - circles[1][0]) > circles[1][2]//No circles too close of each other (horizontal)
                && abs(circles[0][0] - circles[1][0]) <= (circles[0][2]+circles[1][2])*2  ) //No circles too far away
            {
                if (circles[0][0] < circles[1][0])
                {
                    wheels_position.push_back(cv::Vec3f(circles[0][0], circles[0][1], circles[0][2]));
                    wheels_position.push_back(cv::Vec3f(circles[1][0], circles[1][1], circles[1][2]));
                }
                else
                {
                    wheels_position.push_back(cv::Vec3f(circles[1][0], circles[1][1], circles[1][2]));
                    wheels_position.push_back(cv::Vec3f(circles[0][0], circles[0][1], circles[0][2]));
                }
            }
        }
    }

    return wheels_position;
}

cv::Vec3f WheeledRobotPoseEstimator::get3DCoordinates(cv::Point2d point)
{
    cv::Mat disparity = cv::Mat::zeros(cv::Size(inDepth.data.cols, inDepth.data.rows), CV_32FC1);
    disparity.data = inDepth.data.data.arr;

    double fx = inDepth.intrinsic.cameraMatrix.arr[0].arr[0];
    double cx = inDepth.intrinsic.cameraMatrix.arr[0].arr[2];
    double fy = inDepth.intrinsic.cameraMatrix.arr[1].arr[1];
    double cy = inDepth.intrinsic.cameraMatrix.arr[1].arr[2];

    cv::Vec3f point_3d;
    double disparity_value = disparity.at<float>(point.y,point.x);
    if(disparity_value != 0 && disparity_value > (fx*parameters.baseline)/parameters.maxStereoDepth)
    {
        double z = fx * parameters.baseline / disparity_value;
        point_3d[0] = (z / fx) * (point.x - cx);
        point_3d[1] = (z / fy) * (point.y - cy);
        point_3d[2] = z;
    }

    return point_3d;
}

void WheeledRobotPoseEstimator::visualizeResults(std::vector<cv::Vec3f> wheels, std::vector<cv::Point> axisPoints)
{
    if( parameters.visualize )
    {
        cv::Mat img = Converters::FrameToMatConverter().Convert(&inImage);
        cv::cvtColor(img, img, CV_GRAY2RGB);

        cv::Scalar red = cv::Scalar(0,0,255);
        cv::Scalar blue = cv::Scalar(255,0,0);
        cv::Scalar green = cv::Scalar(0,255,0);
        double line_thickness = 2;

        if( wheels.size() == 2 && axisPoints.size() == 5 )
        {
            cv::Point wheel_center_1 = cv::Point(wheels[0][0], wheels[0][1]);
            cv::Point wheel_center_2 = cv::Point(wheels[1][0], wheels[1][1]);
            cv::Point origin = axisPoints[0];
            cv::Point top_left = axisPoints[1];
            cv::Point top_right = axisPoints[2];
            cv::Point bottom_left = axisPoints[3];
            cv::Point bottom_right = axisPoints[4];

            cv::circle( img, wheel_center_1, wheels[0][2], green, line_thickness, cv::LINE_AA);
            cv::circle( img, wheel_center_2, wheels[1][2], green, line_thickness, cv::LINE_AA);

            cv::line( img, wheel_center_1, top_left, green, line_thickness, cv::LINE_AA);
            cv::line( img, wheel_center_2, top_right, green, line_thickness, cv::LINE_AA);
            cv::line( img, top_left, top_right, green, line_thickness, cv::LINE_AA);
            cv::line( img, bottom_left, bottom_right, green, line_thickness, cv::LINE_AA);


            if( m_last_center.x != -1)
            {
                cv::Point end_x = axisPoints[5];
                cv::Point end_z = axisPoints[6];
                cv::arrowedLine(img, origin, end_x, red, line_thickness, cv::LINE_AA);
                cv::arrowedLine(img, origin, end_z, blue, line_thickness, cv::LINE_AA);
            }
        }
        else if (axisPoints.size() == 1)
        {
            cv::Point origin = axisPoints[0];
            cv::circle(img, origin, 5, red, 5);
        }


        cv::namedWindow(parameters.robot+"_TRACKER", CV_WINDOW_NORMAL);
        cv::imshow(parameters.robot+"_TRACKER", img);
        cv::waitKey(10);
    }
}
}
}
}

/** @} */
