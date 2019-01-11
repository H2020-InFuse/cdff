/**
 * @addtogroup DFNs
 * @{
 */

#include "PoseWeighting.hpp"

namespace CDFF
{
namespace DFN
{
namespace WeightingExpert
{

PoseWeighting::PoseWeighting()
: parameters(DEFAULT_PARAMETERS)
{
    m_count = 0;
    m_last_center.x = -1;
    m_last_center.y = -1;
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
    m_kalman_filter->processNoiseCov.at<float>(0) = 10.0f;    //Ex
    m_kalman_filter->processNoiseCov.at<float>(7) = 10.0f;    //Ey
    m_kalman_filter->processNoiseCov.at<float>(14) = 10.0f;   //Ev_x
    m_kalman_filter->processNoiseCov.at<float>(21) = 10.0f;   //Ev_y
    m_kalman_filter->processNoiseCov.at<float>(28) = 1e-2;   //Ew
    m_kalman_filter->processNoiseCov.at<float>(35) = 1e-2;   //Eh

    cv::setIdentity(m_kalman_filter->measurementNoiseCov, cv::Scalar(1e-1));
}

PoseWeighting::~PoseWeighting()
{
}

void PoseWeighting::configure()
{
    if( configurationFilePath.empty() == false )
    {
        parametersHelper.ReadFile(configurationFilePath);
    }
}

const PoseWeighting::PoseWeightingOptionsSet PoseWeighting::DEFAULT_PARAMETERS =
{
    /*numFrames =*/ 4,
    /*maxDistance =*/ 10
};

void PoseWeighting::process()
{
    std::vector<cv::Point> centers;

    if(m_count < parameters.numFrames)
    {
        m_last_centers.clear();
        for(int index = 0; index < inPoses.nCount; index ++)
        {
            m_last_centers.push_back(cv::Point(inPoses.arr[index].pos.arr[0], inPoses.arr[index].pos.arr[1]));
        }

        m_last_center = m_last_centers[0];
        cv::Point measPt = m_last_center;

        //Initialize kalman
        int stateSize = 6;
        int measSize = 4;
        unsigned int type = CV_32F;

        cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
        cv::Mat_<float> measurement(measSize, 1, type);
        measurement.at<float>(0) = measPt.x;
        measurement.at<float>(1) = measPt.y;
        measurement.at<float>(2) = 0;
        measurement.at<float>(3) = 0;


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
        m_count ++;
    }
    else
    {
        centers.clear();
        for(int index = 0; index < inPoses.nCount; index ++)
        {
            centers.push_back(cv::Point(inPoses.arr[index].pos.arr[0], inPoses.arr[index].pos.arr[1]));
        }

        std::vector<double> distances_to_last_output_center;
        std::vector<double> distances_to_last_category_center;
        for( int index = 0; index < centers.size(); index ++ )
        {
            distances_to_last_category_center.push_back(cv::norm(m_last_centers[index] - centers[index]));
            distances_to_last_output_center.push_back(cv::norm(m_last_center - centers[index]));
        }


        cv::Point predictPt;
        cv::Mat prediction = m_kalman_filter->predict();
        predictPt.x = prediction.at<float>(0);
        predictPt.y = prediction.at<float>(1);


        std::vector<double>::iterator result = std::min_element(std::begin(distances_to_last_output_center), std::end(distances_to_last_output_center));
        int index_min_distance = std::distance(std::begin(distances_to_last_output_center), result);
        if( distances_to_last_category_center[index_min_distance] < parameters.maxDistance )
        {
            outPose = inPoses.arr[index_min_distance];
            m_last_center = centers[index_min_distance];

            cv::Point measPt = centers[index_min_distance];
            int measSize = 4;
            unsigned int type = CV_32F;
            cv::Mat_<float> measurement(measSize, 1, type);
            measurement.at<float>(0) = measPt.x;
            measurement.at<float>(1) = measPt.y;
            measurement.at<float>(2) = 0;
            measurement.at<float>(3) = 0;

            m_kalman_filter->correct(measurement);
        }
        else
        {
            outPose.pos.arr[0] = predictPt.x;
            outPose.pos.arr[1] = predictPt.y;
        }

        m_last_centers = centers;
    }
}

}
}
}

/** @} */
