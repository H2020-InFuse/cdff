/**
 * @addtogroup DFNs
 * @{
 */

#include "LidarBasedTracking.hpp"

namespace CDFF
{
namespace DFN
{
namespace LidarBasedTracking
{
/////////////////////////////// Public Methods ////////////////////////////////
// Constructor ****************************************************************
LidarBasedTracking::LidarBasedTracking() :
    m_kf(12,6,0), m_isInitialized(false)
{
    m_parametersHelper.AddParameter<float>(
                "LidarTrackParameters",
                "StandardDeviationtdOrientationProcess",
                m_parameters.stdOrientationProcess,
                DEFAULT_PARAMETERS.stdOrientationProcess);
    m_parametersHelper.AddParameter<float>(
                "LidarTrackParameters",
                "StandardDeviationtdTranslationProcess",
                m_parameters.stdTranslationProcess,
                DEFAULT_PARAMETERS.stdTranslationProcess);
    m_parametersHelper.AddParameter<float>(
                "LidarTrackParameters",
                "StandardDeviationtdOrientationMeasurement",
                m_parameters.stdOrientationMeasurement,
                DEFAULT_PARAMETERS.stdOrientationMeasurement);
    m_parametersHelper.AddParameter<float>(
                "LidarTrackParameters",
                "StandardDeviationtdTranslationMeasurement",
                m_parameters.stdTranslationMeasurement,
                DEFAULT_PARAMETERS.stdTranslationMeasurement);
    m_parametersHelper.AddParameter<float>(
                "LidarTrackParameters",
                "InitialStateCovariance",
                m_parameters.initialCovariance,
                DEFAULT_PARAMETERS.initialCovariance);
    m_parametersHelper.AddParameter<double>(
                "LidarTrackParameters",
                "MaxCorrespondenceDistance",
                m_parameters.maxCorrespondenceDistance,
                DEFAULT_PARAMETERS.maxCorrespondenceDistance);
    m_parametersHelper.AddParameter<int>(
                "LidarTrackParameters",
                "MaximumIterations",
                m_parameters.maximumIterations,
                DEFAULT_PARAMETERS.maximumIterations);
    m_parametersHelper.AddParameter<double>(
                "LidarTrackParameters",
                "TransformationEpsilon",
                m_parameters.transformationEpsilon,
                DEFAULT_PARAMETERS.transformationEpsilon);
    m_parametersHelper.AddParameter<double>(
                "LidarTrackParameters",
                "EuclideanFitnessEpsilon",
                m_parameters.euclideanFitnessEpsilon,
                DEFAULT_PARAMETERS.euclideanFitnessEpsilon);

    configurationFilePath = "";
    m_parameters.targetCloudHasNormals = false;
    asn1SccRigidBodyState_Initialize(&outState);
    asn1SccPointcloud_Initialize(&inSourceCloud);

}

// Destructor *****************************************************************
LidarBasedTracking::~LidarBasedTracking()
{
    delete m_icp;
}

// Configuration **************************************************************
void LidarBasedTracking::configure()
{
    if(configurationFilePath != ""){
        m_parametersHelper.ReadFile(configurationFilePath);
    }
    validateParameters();
}

void LidarBasedTracking::validateParameters()
{
    ASSERT(m_parameters.initialCovariance > 0,
           "LidarBasedTracking Configuration error:"
           "initial state covariance must be strictly positive");
    ASSERT(m_parameters.stdOrientationProcess > 0,
           "LidarBasedTracking configuration error: "
           "process covariance (orientation) must be strictly positive");
    ASSERT(m_parameters.stdTranslationProcess > 0,
           "LidarBasedTracking configuration error: "
           "process covariance (translation) must be strictly positive");
    ASSERT(m_parameters.stdOrientationMeasurement > 0,
           "LidarBasedTracking configuration error: "
           "measurement covariance (orientation) must be strictly positive");
    ASSERT(m_parameters.stdTranslationMeasurement > 0,
           "LidarBasedTracking configuration error: "
           "measurement covariance (translation) must be strictly positive");
    ASSERT(m_parameters.maxCorrespondenceDistance >= 0,
           "LidarBasedTracking Configuration error: "
           "Max Correspondence Distance must be strictly positive");
    ASSERT(m_parameters.maximumIterations >= 0,
           "LidarBasedTracking Configuration error: "
           "Maximum Iterations is negative");
    ASSERT(m_parameters.transformationEpsilon > 0,
           "LidarBasedTracking Configuration error: "
           "Transformation Epsilon cannot be negative");
    ASSERT(m_parameters.euclideanFitnessEpsilon > 0,
           "LidarBasedTracking Configuration error: "
           "Euclidean Fitness Epsilon cannot be negative");
}

const LidarBasedTracking::LidarTrackOptionsSet
LidarBasedTracking::DEFAULT_PARAMETERS =
{
    .stdOrientationProcess = 1.0,
    .stdTranslationProcess = 1.0,
    .stdOrientationMeasurement = 1.0,
    .stdTranslationMeasurement = 1.0,
    .initialCovariance = 1.0,
    .maxCorrespondenceDistance = 0.05,
    .maximumIterations = 50,
    .transformationEpsilon = 1e-8,
    .euclideanFitnessEpsilon = 1.0
};

// Initialization *************************************************************
void LidarBasedTracking::init(const cv::Mat &initialState,
                                  const std::string &modelPath, double scale)
{
    // Initialize Kalman Filter
    validateStateVector(initialState);
    m_kf.statePre = initialState;
    setIdentity(m_kf.errorCovPre,
                cv::Scalar::all(m_parameters.initialCovariance));
    m_timeOfLastMeasurement = -1;

    // Initialize ICP
    loadPointCloud(modelPath, scale);
    cvMatToEigenTransform(initialState, m_transformGuess);

    m_isInitialized = true;
}

// Processing *****************************************************************
void LidarBasedTracking::process()
{
    ASSERT(m_isInitialized, "LidarBasedTracking Error: process() was "
                            "called before the instance was initialized.");

    long long currentTime; // Time of new measurement
    float dt; // Time since last measurement
    Eigen::Matrix4f icpTransform; // Result of the ICP
    // Measurement matrix used by the KF (correction step)
    cv::Mat measurement(6, 1, CV_32F, cv::Scalar(0));
    // State vector after the prediction step
    cv::Mat predictedState(12, 1, CV_32F, cv::Scalar(0));
    // State covariance matrix after the prediction step);
    cv::Mat predictedCov(12, 12, CV_32F, cv::Scalar(0));
    // State vector after the correction step
    cv::Mat correctedState(12, 1, CV_32F, cv::Scalar(0));
    // State covariance matrix after the correction step
    cv::Mat correctedCov(12, 12, CV_32F, cv::Scalar(0));

    // Read input port
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloudNNPtr =
            m_pointCloudToPclPointCloud.Convert(&inSourceCloud);
    pcl::PointCloud<sourcePoint> sourceCloud;
    pcl::copyPointCloud(*sourceCloudNNPtr, sourceCloud);
    pcl::PointCloud<targetPoint>::ConstPtr targetCloudPtr
            (new pcl::PointCloud<targetPoint> (m_targetCloud));
    pcl::PointCloud<sourcePoint>::ConstPtr sourceCloudPtr
            (new pcl::PointCloud<sourcePoint> (sourceCloud));
    validateCloud(sourceCloudPtr);

    // Get time of new measurement
    /* The time step dt used in the KF prediction step is the difference between
     * the current measurement and the previous one. The prediction step is
     * therefore performed only after a new measurement is available.
     */
    currentTime = inSourceCloud.metadata.timeStamp.microseconds;

    // Perform prediction step
    /* For the first loop (when only one measurement is available), we are not
     * able to compute dt, and thus to perform the prediction step. In this case
     * we skip the prediction step and perform the update with the available
     * measurement and the initial state (set by the method init).
     */
    if(m_timeOfLastMeasurement >= 0)
    {
        dt = (currentTime - m_timeOfLastMeasurement) * 1e-6;
        predict(dt, predictedState, predictedCov);
        cvMatToEigenTransform(predictedState, m_transformGuess);
    }

    // Perform ICP
    /* ICP estimates the transform between a known point cloud (m_targetCloud)
     * and an unknown point cloud (inSourceCloud). An transform guess is provided
     * to help the ICP algorithm to converge in a global optimum. In our case,
     * that guess is the predicted state of our KF.
     */
    computeTransform(sourceCloudPtr, targetCloudPtr,
                     m_transformGuess, icpTransform);
    eigenTransformToCvMat(icpTransform, measurement);

    // Perform correction step
    correct(measurement, correctedState, correctedCov);
    // Write state vector to output port
    cvMatToRigidBodyState(correctedState, correctedCov, outState);
    outState.timestamp = inSourceCloud.metadata.timeStamp;
    outState.sourceFrame = inSourceCloud.metadata.frameId;
    std::string targetFrame = "targetFrame";
    outState.targetFrame.nCount = static_cast<int>(targetFrame.size() + 1);
    memcpy(outState.targetFrame.arr, targetFrame.data(),
           static_cast<size_t>(outState.targetFrame.nCount));

    // Prepare next loop
    m_timeOfLastMeasurement = inSourceCloud.metadata.timeStamp.microseconds;
}

////////////////////////////// Private Methods ////////////////////////////////
// Kalman Filter Prediction ***************************************************
bool LidarBasedTracking::predict(float dt,
                                     cv::Mat &predictedState,
                                     cv::Mat &predictedCov)
{
    cv::Mat Q(12, 12, CV_32F, cv::Scalar(0)); // Process noise matrix
    float stdTranslation; // Standard deviation of velocity noise
    float stdOrientation; // Standard deviation of angular velocity noise
    stdOrientation = m_parameters.stdOrientationProcess;
    stdTranslation = m_parameters.stdTranslationProcess;

    // Fill process noise matrix Q
    // (see https://en.wikipedia.org/wiki/Kalman_filter)
    for (int i=0;i<3;i++)
    {
        Q.at<float>(i,i) = 0.25 * std::pow(dt,4)
                * std::pow(stdTranslation,2);
        Q.at<float>(i+3,i+3) = 0.25 * std::pow(dt,4)
                * std::pow(stdOrientation,2);
        Q.at<float>(i+6,i) = 0.5 * std::pow(dt,3)
                * std::pow(stdTranslation,2);
        Q.at<float>(i+9,i+3) = 0.5 * std::pow(dt,3)
                * std::pow(stdOrientation,2);
        Q.at<float>(i,i+6) = 0.5 * std::pow(dt,3)
                * std::pow(stdTranslation,2);
        Q.at<float>(i+3,i+9) = 0.5 * std::pow(dt,3)
                * std::pow(stdOrientation,2);
        Q.at<float>(i+6,i+6) = std::pow(dt,2)
                * std::pow(stdTranslation,2);
        Q.at<float>(i+9,i+9) = std::pow(dt,2)
                * std::pow(stdOrientation,2);
    }

    // Update Kalman Filter with the new process noise matrix
    m_kf.processNoiseCov = Q;

    // Compute the new transition matrix
    setIdentity(m_kf.transitionMatrix, cv::Scalar::all(1));
    for(int i = 0; i < 6; i++)
    {
        m_kf.transitionMatrix.at<float>(i,6+i)=dt;
    }

    // Perform prediction
    predictedState = m_kf.predict();
    predictedCov = m_kf.errorCovPre;

    return 1;
}

// Kalman Filter Correction ***************************************************
bool LidarBasedTracking::correct(const cv::Mat &measurement,
                                     cv::Mat &correctedState,
                                     cv::Mat &correctedCov)
{
    float stdOrientation = m_parameters.stdOrientationMeasurement;
    float stdTranslation = m_parameters.stdTranslationMeasurement;

    // Compute measurement noise covariance matrix
    m_kf.measurementNoiseCov.setTo(cv::Scalar(0));
    for(int i = 0; i < 3; i++)
    {
        m_kf.measurementNoiseCov.at<float>(i,i) =
                std::pow(stdTranslation,2);
        m_kf.measurementNoiseCov.at<float>(i+3,i+3) =
                std::pow(stdOrientation,2);
    }

    // Compute measurement matrix
    m_kf.measurementMatrix.setTo(cv::Scalar(0));
    for(int i = 0; i < 6; i++)
    {
        m_kf.measurementMatrix.at<float>(i,i) = 1;
    }

    // Perform correction
    correctedState = m_kf.correct(measurement);
    correctedCov = m_kf.errorCovPost;

    return 1;
}

// Loading source point cloud *************************************************
bool LidarBasedTracking::loadPointCloud(std::string modelPath,double scale)
{
    std::string fileType;
    fileType = modelPath.substr(modelPath.find_last_of(".") + 1,
                                modelPath.size());
    if(fileType == "pcd")
    {
        if (pcl::io::loadPCDFile(modelPath, m_targetCloud) < 0)
        {
            PCL_ERROR ("LidarBasedTracking Error: "
                       "error loading point cloud %s.\n", modelPath);
            return -1;
        }
    }
    else if(fileType == "ply")
    {
        if (pcl::io::loadPLYFile(modelPath, m_targetCloud) < 0)
        {
            PCL_ERROR ("LidarBasedTracking Error: "
                       "error loading point cloud %s.\n", modelPath);
            return -1;
        }
    }
    else
    {
        PCL_ERROR ("LidarBasedTracking Error: "
                   "type of %s unknown.\n", modelPath);
        return -1;
    }

    if (scale != 1)
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform (0,0) = transform (0,0) * scale;
        transform (1,1) = transform (1,1) * scale;
        transform (2,2) = transform (2,2) * scale;
        pcl::transformPointCloud(m_targetCloud, m_targetCloud, transform);
    }
    validateCloud(&m_targetCloud);

    if (m_parameters.targetCloudHasNormals)
    {
        m_icp = new pcl::IterativeClosestPointWithNormals
                <sourcePoint,targetPoint>();
    }
    else
    {
        m_icp = new pcl::IterativeClosestPoint<sourcePoint, targetPoint>();
    }

    return 1;
}

// Performing ICP *************************************************************
bool LidarBasedTracking::computeTransform(
        pcl::PointCloud<sourcePoint>::ConstPtr sourceCloud,
        pcl::PointCloud<targetPoint>::ConstPtr targetCloud,
        const Eigen::Matrix4f &transformGuess,
        Eigen::Matrix4f &transformOut)
{
    // Setup PCL's ICP algorithm
    m_icp->setInputSource(sourceCloud);
    m_icp->setInputTarget(targetCloud);
    m_icp->setMaxCorrespondenceDistance(m_parameters.maxCorrespondenceDistance);
    m_icp->setMaximumIterations(m_parameters.maximumIterations);
    m_icp->setTransformationEpsilon(m_parameters.transformationEpsilon);
    m_icp->setEuclideanFitnessEpsilon(m_parameters.euclideanFitnessEpsilon);

    // Setup output
    pcl::PointCloud<sourcePoint> outputCloud;

    // Run ICP
    m_icp->align(outputCloud, transformGuess);

    // Check convergence
    bool outSuccess = m_icp->hasConverged();

    if (outSuccess)
    {
        transformOut = m_icp->getFinalTransformation();

        // Problem with getFinalTransformation : determinant of rotation matrix
        // not equal to 1. Need to find the nearest orthogonal matrix
        // (svd decomposition method)
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(transformOut.block(0,0,3,3),
                                              Eigen::ComputeThinU |
                                              Eigen::ComputeThinV);
        transformOut.block(0,0,3,3) = svd.matrixU() * svd.matrixV().transpose();
    }
    else
    {
        // ICP didn't converge
        transformOut.setZero();
    }
    return outSuccess;
}

// OpenCv <-> Eigen matrix conversion *****************************************
void LidarBasedTracking::cvMatToEigenTransform(const cv::Mat &inMat,
                                                   Eigen::Matrix4f &outMat)
{
    ASSERT(inMat.cols == 1 && inMat.rows >= 6,
           "LidarBasedTracking Conversion Error : "
           "cv::Mat inMat must be a vector with at least 6 rows");
    Eigen::Vector3f euler;
    Eigen::Matrix3f rotationMatrix;
    outMat = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; i++)
    {
        outMat(i,3) = inMat.at<float>(i,0);
        euler(i) = inMat.at<float>(i+3,0);
    }
    rotationMatrix = Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitZ());
    outMat.block<3,3>(0,0) = rotationMatrix;
}

void LidarBasedTracking::eigenTransformToCvMat(const Eigen::Matrix4f &inMat,
                                                   cv::Mat &outMat)
{
    ASSERT(outMat.cols == 1 && outMat.rows >= 6,
           "LidarBasedTracking Conversion Error : "
           "cv::Mat outMat must be a vector with at least 6 rows");
    Eigen::Vector3f euler;
    euler = inMat.block<3,3>(0,0).eulerAngles(0,1,2);
    for (int i = 0; i < 3; i++)
    {
        outMat.at<float>(i,0) = inMat(i,3);
        outMat.at<float>(i+3,0) = euler(i);
    }
}

// OpenCv -> asn1SccRigidBodyState conversion **********************************
void LidarBasedTracking::cvMatToRigidBodyState(const cv::Mat &state,
                           const cv::Mat &covariance,
                           asn1SccRigidBodyState &outState)
{
    // Check inputs
    ASSERT(state.rows >= 12 && state.cols >= 1,
           "MatToRigidBodyStateConverter Error: "
           "state matrix must be of size 12*1");
    ASSERT(covariance.rows >= 12 && covariance.cols >= 12,
           "MatToRigidBodyStateConverter Error, "
           "covariance matrix must be of size 12*12");
    ASSERT(state.type() == CV_32FC1,
           "MatToRigidBodyStateConverter Error, "
           "an invalid state matrix type was passed");
    ASSERT(covariance.type() == CV_32FC1,
           "MatToRigidBodyStateConverter Error, "
           "an invalid covariance matrix type was passed");

    // Variables used for quaternion <-> euler conversion
    Eigen::Quaterniond quaternion;
    Eigen::Vector3d euler;

    for (int i = 0; i < 3; i++)
    {
        outState.pos.arr[i] = state.at<float>(i);
        euler(i) = state.at<float>(i+3);
        outState.velocity.arr[i] =
                state.at<float>(i+6);
        outState.angular_velocity.arr[i] =
                state.at<float>(i+9);
    }

    // Convert euler angles to quaternion and write to output port
    quaternion = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitZ());
    quaternion.normalize();
    outState.orient.arr[0] = quaternion.w();
    outState.orient.arr[1] = quaternion.x();
    outState.orient.arr[2] = quaternion.y();
    outState.orient.arr[3] = quaternion.z();

    // Write state vector covariance to output port
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            outState.cov_position.arr[i].arr[j] =
                    covariance.at<float>(i,j);
            outState.cov_orientation.arr[i].arr[j] =
                    covariance.at<float>(i+3,j+3);
            outState.cov_velocity.arr[i].arr[j] =
                    covariance.at<float>(i+6,j+6);
            outState.cov_angular_velocity.arr[i].arr[j] =
                    covariance.at<float>(i+9,j+9);
        }
    }
}

// Validation ******************************************************************
void LidarBasedTracking::validateInputs(
        pcl::PointCloud<sourcePoint>::ConstPtr sourceCloud,
        pcl::PointCloud<targetPoint>::ConstPtr targetCloud)
{
    validateCloud(sourceCloud);
    validateCloud(targetCloud);
}

template<typename cloudTypePtr>
void LidarBasedTracking::validateCloud(cloudTypePtr cloud)
{
    for (unsigned pointIndex = 0;pointIndex < cloud->points.size();pointIndex++)
    {
        const auto &point = cloud->points.at(pointIndex);
        ASSERT_EQUAL(point.x, point.x, "IcpPLY Error, Cloud contains an NaN point");
        ASSERT_EQUAL(point.y, point.y, "IcpPLY Error, Cloud contains an NaN point");
        ASSERT_EQUAL(point.z, point.z, "IcpPLY Error, Cloud contains an NaN point");
    }
}

void LidarBasedTracking::validateStateVector(const cv::Mat &state)
{
    ASSERT(state.rows == 12 && state.cols == 1, "LidarBasedTracking Error: "
                                                "State vector must be of size 12*1");
}

} // namespace LidarBasedTracking
} // namespace DFN
} // namespace CDFF

/** @} */
