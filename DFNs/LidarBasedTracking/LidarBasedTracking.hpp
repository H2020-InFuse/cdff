/**
 * @addtogroup DFNs
 * @{
 */

#ifndef LIDARBASEDTRACKING_LIDARBASEDTRACKING_HPP
#define LIDARBASEDTRACKING_LIDARBASEDTRACKING_HPP

// Includes ////////////////////////////////////////////////////////////////////
#include "LidarBasedTrackingInterface.hpp"

#include <Helpers/ParametersListHelper.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>

#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/impl/io.hpp>

#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

#include <Macros/YamlcppMacros.hpp>
#include <Errors/Assert.hpp>

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <memory>

// Namespace ///////////////////////////////////////////////////////////////////
namespace CDFF
{
namespace DFN
{
namespace LidarBasedTracking
{
typedef pcl::PointNormal sourcePoint;
typedef pcl::PointNormal targetPoint;

// Class Definition ////////////////////////////////////////////////////////////
/**
     * DFN that tracks a model in a point cloud. For a given source cloud
     * (asn1SccPointCloud) and target cloud (pcl::PointCloud) returns the
     * transformation from source cloud frame to target cloud frame.
     */
class LidarBasedTracking : public LidarBasedTrackingInterface
{
    // Attributes **************************************************************
public:
    struct LidarTrackOptionsSet
    {
        // Kalman filter's parameters
        float stdOrientationProcess;
        float stdTranslationProcess;
        float stdOrientationMeasurement;
        float stdTranslationMeasurement;
        float initialCovariance;

        // ICP's parameters
        double maxCorrespondenceDistance;
        int maximumIterations;
        double transformationEpsilon;
        double euclideanFitnessEpsilon;
        bool targetCloudHasNormals;
    };
    LidarTrackOptionsSet m_parameters;
    Helpers::ParametersListHelper m_parametersHelper;
    // Following macro needed when the class has a fixed size Eigen type
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        static const LidarTrackOptionsSet DEFAULT_PARAMETERS;
    Converters::PointCloudToPclPointCloudConverter
    m_pointCloudToPclPointCloud;
    cv::KalmanFilter m_kf;
    pcl::PointCloud<targetPoint> m_targetCloud;
    bool m_isInitialized = false;
    long long m_timeOfLastMeasurement = 0;
    Eigen::Matrix4f m_transformGuess; // Transform guess used by the ICP
    pcl::IterativeClosestPoint<sourcePoint, targetPoint> *m_icp;

    // Methods *****************************************************************
public:
    LidarBasedTracking();
    virtual ~LidarBasedTracking();
    virtual void configure();
    void validateParameters();
    virtual void process();
    /**
         * @brief Reinitializes the Kalman Filter and the ICP
         * @param[in] initialState : initial state of the Kalman Filter
         * @param[in] modelPath : path to the traget point cloud for the ICP
         * @param[in] scale : scale of the target point cloud
         */
    void init(const cv::Mat &initialState,
              const std::string &modelPath, double scale);
private:
    // Kalman filter
    bool predict(float dt,
                 cv::Mat &predictedState,
                 cv::Mat &predictedCov);
    bool correct(const cv::Mat &measurement,
                 cv::Mat &correctedState,
                 cv::Mat &correctedCov);
    // ICP
    bool loadPointCloud(std::string modelPath, double scale);
    bool computeTransform(
            pcl::PointCloud<sourcePoint>::ConstPtr sourceCloud,
            pcl::PointCloud<targetPoint>::ConstPtr targetCloud,
            const Eigen::Matrix4f &transformGuess,
            Eigen::Matrix4f &transformOut);
    // Conversion
    void cvMatToEigenTransform(const cv::Mat &inMat,
                               Eigen::Matrix4f &outMat);
    void eigenTransformToCvMat(const Eigen::Matrix4f &inMat,
                               cv::Mat &outMat);
    void cvMatToRigidBodyState(const cv::Mat &state,
                               const cv::Mat &covariance,
                               asn1SccRigidBodyState &outState);
    // Validation
    void validateInputs(
            pcl::PointCloud<sourcePoint>::ConstPtr sourceCloud,
            pcl::PointCloud<targetPoint>::ConstPtr targetCloud);
    template<typename cloudTypePtr>
    void validateCloud(const cloudTypePtr cloud);
    void validateStateVector(const cv::Mat &state);
    // Configuration
    void configure(const YAML::Node& configurationNode);
};

} // namespace LidarBasedTracking
} // namespace DFN
} // namespace CDFF

#endif // LIDARBASEDTRACKING_LIDARBASEDTRACKING_HPP

/** @} */
