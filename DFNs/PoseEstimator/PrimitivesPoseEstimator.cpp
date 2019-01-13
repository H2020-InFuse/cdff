/**
 * @addtogroup DFNs
 * @{
 */

#include <Converters/FrameToMatConverter.hpp>
#include "PrimitivesPoseEstimator.hpp"

namespace Converters
{
    std::vector<std::vector<double>> Convert(const asn1SccVectorXdSequence & inPrimitives)
    {
        std::vector<std::vector<double>> primitives;
        for(auto i = 0; i < inPrimitives.nCount; i ++ )
        {
            std::vector<double> data;
            for(auto j = 0; j < inPrimitives.arr[i].nCount; j ++ )
        {
            data.push_back(inPrimitives.arr[i].arr[j]);
        }
            primitives.push_back(data);
        }
        return primitives;
    }
}

namespace CDFF
{
namespace DFN
{
namespace PoseEstimator
{

PrimitivesPoseEstimator::PrimitivesPoseEstimator()
    : m_count(0)
{
    configurationFilePath = "";
    parametersHelper.AddParameter<int>("GeneralParameters", "NumFrames", parameters.numFrames, DEFAULT_PARAMETERS.numFrames);
    parametersHelper.AddParameter<double>("GeneralParameters", "MaxDistance", parameters.maxDistance, DEFAULT_PARAMETERS.maxDistance);


    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;
    unsigned int type = CV_32F;
    m_kalman_filter.reset(new cv::KalmanFilter(stateSize, measSize, contrSize, type));

    cv::setIdentity(m_kalman_filter->transitionMatrix);

    m_kalman_filter->measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    m_kalman_filter->measurementMatrix.at<float>(0) = 1.0f;
    m_kalman_filter->measurementMatrix.at<float>(7) = 1.0f;
    m_kalman_filter->measurementMatrix.at<float>(16) = 1.0f;
    m_kalman_filter->measurementMatrix.at<float>(23) = 1.0f;

    cv::setIdentity(m_kalman_filter->processNoiseCov, cv::Scalar(1e-2));
    m_kalman_filter->processNoiseCov.at<float>(0) = 5.0f;    //Ex
    m_kalman_filter->processNoiseCov.at<float>(7) = 5.0f;    //Ey
    m_kalman_filter->processNoiseCov.at<float>(14) = 5.0f;   //Ev_x
    m_kalman_filter->processNoiseCov.at<float>(21) = 5.0f;   //Ev_y
    m_kalman_filter->processNoiseCov.at<float>(28) = 1e-2;   //Ew
    m_kalman_filter->processNoiseCov.at<float>(35) = 1e-2;   //Eh

    cv::setIdentity(m_kalman_filter->measurementNoiseCov, cv::Scalar(1e-1));
}

PrimitivesPoseEstimator::~PrimitivesPoseEstimator()
{
}

const PrimitivesPoseEstimator::PrimitivesPoseEstimatorOptionsSet PrimitivesPoseEstimator::DEFAULT_PARAMETERS =
{
        /*numFrames =*/ 2,
        /*maxDistance =*/ 8
};

void PrimitivesPoseEstimator::configure()
{
    if(configurationFilePath.empty() == false)
    {
        parametersHelper.ReadFile(configurationFilePath);
    }
}

void PrimitivesPoseEstimator::process()
{
    asn1SccPosesSequence_Initialize(&outPoses);

    cv::Mat inputImage = Converters::FrameToMatConverter().Convert(&inImage);
    cv::Mat inputDepth = cv::Mat::zeros(cv::Size(inDepth.data.cols, inDepth.data.rows), CV_32FC1);
    inputDepth.data = inDepth.data.data.arr;
    m_primitives = Converters::Convert(inPrimitives);

    SortPrimitives();

    cv::Point point = PredictPosition();
    Eigen::Quaterniond quaternion = PredictOrientation(inputImage, point, inputDepth);

    outPoses.nCount = 1;
    outPoses.arr[0].pos.arr[0] = point.x;
    outPoses.arr[0].pos.arr[1] = point.y;
    outPoses.arr[0].pos.arr[2] = inputDepth.at<float>(point);

    outPoses.arr[0].orient.arr[0] = quaternion.w();
    outPoses.arr[0].orient.arr[1] = quaternion.x();
    outPoses.arr[0].orient.arr[2] = quaternion.y();
    outPoses.arr[0].orient.arr[3] = quaternion.z();

    m_primitives.clear();
}

void PrimitivesPoseEstimator::SortPrimitives()
{
    auto last_point = m_last_point;
    std::sort(m_primitives.begin(), m_primitives.end(), [last_point](std::vector<double> a, std::vector<double> b) {
        auto distance_a = cv::norm(last_point - cv::Point(a[0], a[1]));
        auto distance_b = cv::norm(last_point - cv::Point(b[0], b[1]));
        return distance_a < distance_b;
    });
}

void PrimitivesPoseEstimator::InitializeKalman(cv::Point measPt)
{
    int stateSize = 6;
    int measSize = 4;
    unsigned int type = CV_32F;

    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    cv::Mat_<float> measurement(measSize, 1, type);
    measurement.at<float>(0) = measPt.x;
    measurement.at<float>(1) = measPt.y;
    measurement.at<float>(2) = m_primitives[0][2];
    measurement.at<float>(3) = m_primitives[0][2];


    m_kalman_filter->errorCovPre.at<float>(0) = 1; // px
    m_kalman_filter->errorCovPre.at<float>(7) = 1; // px
    m_kalman_filter->errorCovPre.at<float>(14) = 1;
    m_kalman_filter->errorCovPre.at<float>(21) = 1;
    m_kalman_filter->errorCovPre.at<float>(28) = 1; // px
    m_kalman_filter->errorCovPre.at<float>(35) = 1; // px

    state.at<float>(0) = measurement.at<float>(0);
    state.at<float>(1) = measurement.at<float>(1);
    state.at<float>(2) = 0;
    state.at<float>(3) = 0;
    state.at<float>(4) = measurement.at<float>(2);
    state.at<float>(5) = measurement.at<float>(3);

    m_kalman_filter->statePost = state;
}

cv::Point PrimitivesPoseEstimator::PredictPosition()
{
    cv::Point predictPt;
    if (m_primitives.empty())
    {
        return predictPt;
    }

    cv::Point measPt(m_primitives[0][0], m_primitives[0][1]);

    if (m_count < parameters.numFrames)
    {
        InitializeKalman(measPt);
        m_last_point.x = measPt.x;
        m_last_point.y = measPt.y;
        m_count++;
    }
    else
    {
        cv::Mat prediction = m_kalman_filter->predict();
        predictPt.x = prediction.at<float>(0);
        predictPt.y = prediction.at<float>(1);

        int measSize = 4;
        unsigned int type = CV_32F;
        cv::Mat_<float> measurement(measSize, 1, type);
        measurement.at<float>(0) = measPt.x;
        measurement.at<float>(1) = measPt.y;
        measurement.at<float>(2) = m_primitives[0][2];
        measurement.at<float>(3) = m_primitives[0][3];


        if (cv::norm(m_last_point - cv::Point(measPt.x, measPt.y)) < parameters.maxDistance)
        {
            m_kalman_filter->correct(measurement);
        }

        m_last_point.x = measPt.x;
        m_last_point.y = measPt.y;

        m_count++;
    }

    return predictPt;
}

Eigen::Quaterniond PrimitivesPoseEstimator::PredictOrientation(const cv::Mat& inputImage, cv::Point point, const cv::Mat& inputDepth)
{
    if( m_primitives.empty() )
    {
        return Eigen::Quaterniond();
    }

    Eigen::Vector3d center(point.x, point.y,inputDepth.at<float>(point));
    Eigen::Vector3d vec1 = Eigen::Vector3d(point.x+1, point.y, inputDepth.at<float>(cv::Point(point.x+1, point.y))) - center;
    Eigen::Vector3d vec2 = Eigen::Vector3d(point.x, point.y+1, inputDepth.at<float>(cv::Point(point.x, point.y+1))) - center;

    Eigen::Vector3d normal = vec1.cross(vec2);
    normal.normalize();

    Eigen::Quaterniond quaternion = quaternion.FromTwoVectors(normal, Eigen::Vector3d(1, 0, 0));
    quaternion.normalize();
    return quaternion;
}

}
}
}

/** @} */
