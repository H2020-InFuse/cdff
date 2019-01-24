/**
 * @addtogroup DFNs
 * @{
 */

#include "WheeledRobotPoseEstimator.hpp"
#include "KFHelper.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/video/tracking.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <Converters/FrameToMatConverter.hpp>

#include <iostream>

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
    : parameters(DEFAULT_PARAMETERS)
{
    m_last_left_wheel_x = -1;
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
    prevCovInnovTrace = 0;

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
    std::vector<cv::Vec3f> robot_wheels = filterRobotWheels();

    std::vector<double> prevQuat = {prevEstimatedPose.orient.arr[0],
                                    prevEstimatedPose.orient.arr[1],
                                    prevEstimatedPose.orient.arr[2],
                                    prevEstimatedPose.orient.arr[3]};

    if(robot_wheels.size()>=2)
    {
        extractRobotPoints(robot_wheels);
        // ---------- CASE 1/3 : wheels position available ----------
        visualizeResults();
        if(m_last_left_wheel_x != -1)
        {
            outputPose(m_robot_points.robot_center_3D, extractOrientation());

            // good confidence in measurements (TODO : lower for orientation )
            cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-3));
            estimatedPose = estimatePose(KF, outPoses.arr[0], measurements, 0, 0);
        }
        else
        {
            // ---------- discard orientation : measure position only
            // TODO : check orientation is not taken in count (otherwise floor its confidence)
            outputPose(m_robot_points.robot_center_3D, prevQuat);
            estimatedPose = estimatePose(KF, outPoses.arr[0], measurements, 0, 1);
        }

        if(abs(m_last_left_wheel_x - m_robot_points.left_wheel.center[0]) >=0.05 || m_last_left_wheel_x == -1)
        {
            m_last_left_wheel_x = m_robot_points.left_wheel.center[0];
        }
    }
    else
    {
        // extract centroid, display, extract position only

        m_robot_points.robot_center_px = extractForegroundCentroid(inputImage);
        m_robot_points.robot_center_3D = get3DCoordinates(m_robot_points.robot_center_px);
        visualizeResults();

        // if backup measurement allowed and available
        if (m_robot_points.robot_center_px.x > 0 && m_robot_points.robot_center_px.y > 0 && KFparameters.backupMeasurement == true)
        {
            outputPose(m_robot_points.robot_center_3D, prevQuat);
            // ---------- CASE 2/3 : foreground centroid available ----------
            // get blob center 3D coords
            asn1SccPose poseFromCentroid;
            asn1SccPose_Initialize(&poseFromCentroid);
            m_robot_points.robot_center_3D = get3DCoordinates(m_robot_points.robot_center_px);
            poseFromCentroid.pos.arr[0] = m_robot_points.robot_center_3D[0];
            poseFromCentroid.pos.arr[1] = m_robot_points.robot_center_3D[1];
            poseFromCentroid.pos.arr[2] = m_robot_points.robot_center_3D[2];

            // TODO : check orientation is not taken in count (otherwise floor its confidence)
            // lower confidence in measurements (higher measurement noise)
            cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
            estimatedPose = estimatePose(KF, poseFromCentroid, measurements, 0, 1);
        }
        else if (prevEstimatedPose.pos.arr[0] !=0 && prevEstimatedPose.pos.arr[1] !=0)
        {
            // ---------- CASE 3/3 : no measures available, prevPose != 0, KF converged -> predict ----------
            estimatedPose = estimatePose(KF, prevEstimatedPose, measurements, 1, 1);
        }
    }



    converged = checkKFConvergence(KF, prevCovInnovTrace, 1e-4);
    if (converged)
    {
        outPoses.arr[0].pos = estimatedPose.pos;
    }
    prevEstimatedPose = outPoses.arr[0];
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

std::vector<double> WheeledRobotPoseEstimator::extractOrientation()
{
    // get rover direction vector w.r.t xUnitVector in image frame (x right, y down, z in)
    cv::Vec3f xUnitVec = cv::Vec3f(1,0,0);
    xUnitVec = cv::normalize(xUnitVec);
    cv::Vec3f rover_heading =  m_robot_points.right_wheel.center - m_robot_points.left_wheel.center;
    cv::Vec3f rover_heading_normalized = cv::normalize(rover_heading);

    Eigen::Quaternionf qx;
    cv::Vec3f a = xUnitVec.cross(rover_heading_normalized);
    qx.x() = a[0];
    qx.y() = a[1];
    qx.z() = a[2];
    qx.w() = xUnitVec.dot(rover_heading_normalized);

    cv::Vec3f zUnitVec = cv::Vec3f(0,0,1);
    cv::Vec3f up_heading = m_robot_points.left_wheel.up - m_robot_points.left_wheel.down;
    cv::Vec3f up_heading_normalized = cv::normalize(up_heading);

    double angle_around_x = acos(zUnitVec.dot(up_heading_normalized));

    Eigen::Quaternionf rotation_around_x = Eigen::AngleAxisf(angle_around_x, Eigen::Vector3f::UnitX())
                                           * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
                                           * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = qx * rotation_around_x;
    if ((m_last_left_wheel_x - m_robot_points.left_wheel.center[0]) > 0)
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
                output_point.y += boundRect.height/5 ;
            }
            else if(parameters.robot==robots::MANA)
            {
                max_diff = 50;
                correction = boundRect.width/4;
                output_point.y += boundRect.height/5 ;
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

void WheeledRobotPoseEstimator::extractRobotPoints(std::vector<cv::Vec3f> wheels)
{
    if(wheels.size() >= 2)
    {
        cv::Point wheel_center_1 = cv::Point(wheels[0][0], wheels[0][1]);
        cv::Point wheel_center_2 = cv::Point(wheels[1][0], wheels[1][1]);

        double distance_between_wheels = cv::norm(wheel_center_1 - wheel_center_2);
        double h = wheels[0][2]/4;
        double alpha = asin((wheel_center_1.y - wheel_center_2.y) / distance_between_wheels);
        double beta = M_PI / 2 - alpha;

        cv::Point top_left, top_right, bottom_left, bottom_right;
        top_left.x = wheel_center_1.x - cos(beta) * h;
        top_left.y = wheel_center_1.y - sin(beta) * h;
        top_right.x = wheel_center_2.x - cos(beta) * h;
        top_right.y = wheel_center_2.y - sin(beta) * h;

        bottom_left.x = wheel_center_1.x + cos(beta) * h;
        bottom_left.y = wheel_center_1.y + sin(beta) * h;
        bottom_right.x = wheel_center_2.x + cos(beta) * h;
        bottom_right.y = wheel_center_2.y + sin(beta) * h;

        m_robot_points.left_wheel.center = get3DCoordinates(wheel_center_1);
        m_robot_points.left_wheel.center2D = wheel_center_1;
        m_robot_points.right_wheel.center = get3DCoordinates(wheel_center_2);
        m_robot_points.right_wheel.center2D = wheel_center_2;
        m_robot_points.left_wheel.up = get3DCoordinates(top_left);
        m_robot_points.right_wheel.up = get3DCoordinates(top_right);
        m_robot_points.left_wheel.down = get3DCoordinates(bottom_left);
        m_robot_points.right_wheel.down = get3DCoordinates(bottom_right);

        m_robot_points.robot_center_3D[0] = (m_robot_points.left_wheel.center[0] + m_robot_points.right_wheel.center[0]) / 2.0;
        m_robot_points.robot_center_3D[1] = (m_robot_points.left_wheel.center[1] + m_robot_points.right_wheel.center[1]) / 2.0;
        m_robot_points.robot_center_3D[2] = (m_robot_points.left_wheel.center[2] + m_robot_points.right_wheel.center[2]) / 2.0;

        m_robot_points.robot_center_px.x = wheel_center_1.x + (wheel_center_2.x - wheel_center_1.x) / 2.0;
        m_robot_points.robot_center_px.y = wheel_center_1.y + (wheel_center_2.y - wheel_center_1.y) / 2;

    }
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
            if ( abs(circles[0][1] - circles[1][1]) <= circles[0][2] )
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

cv::Vec3f WheeledRobotPoseEstimator::get3DCoordinates(cv::Point point)
{
    cv::Vec3f point_3d;

    if( point.x > 0 && point.y > 0 )
    {
        cv::Mat disparity = cv::Mat::zeros(cv::Size(inDepth.data.cols, inDepth.data.rows), CV_32FC1);
        disparity.data = inDepth.data.data.arr;

        double fx = inDepth.intrinsic.cameraMatrix.arr[0].arr[0];
        double cx = inDepth.intrinsic.cameraMatrix.arr[0].arr[2];
        double fy = inDepth.intrinsic.cameraMatrix.arr[1].arr[1];
        double cy = inDepth.intrinsic.cameraMatrix.arr[1].arr[2];


        double disparity_value = disparity.at<float>(point);
        double z = fx * parameters.baseline / disparity_value;
        if( parameters.robot == robots::SHERPA ) //In the case of sherpa, we're using a depth map instead of a disparity image
        {
            z = disparity_value;
        }

        double Dmax = parameters.maxStereoDepth;

        if(disparity_value != 0 &&  z < Dmax)
        {
            point_3d[0] = (z / fx) * (point.x - cx);
            point_3d[1] = (z / fy) * (point.y - cy);
            point_3d[2] = z;
        }
    }

    return point_3d;
}

double WheeledRobotPoseEstimator::getDisparityAroundPoint(cv::Point point, const cv::Mat & disparity)
{
    double disparity_value = disparity.at<float>(point.y,point.x);
    if( disparity_value == 0 ) {
        disparity_value = disparity.at<float>(point.y, point.x + 1);
        if (disparity_value == 0) {
            disparity_value = disparity.at<float>(point.y, point.x - 1);
            if (disparity_value == 0) {
                disparity_value = disparity.at<float>(point.y + 1, point.x);
                if (disparity_value == 0) {
                    disparity_value = disparity.at<float>(point.y - 1, point.x);
                }
            }
        }
    }
    return disparity_value;
}

void WheeledRobotPoseEstimator::visualizeResults()
{
    if( parameters.visualize )
    {
        cv::Mat img = Converters::FrameToMatConverter().Convert(&inImage);
        cv::cvtColor(img, img, CV_GRAY2RGB);

        cv::Scalar red = cv::Scalar(0,0,255);
        cv::circle(img, m_robot_points.robot_center_px, 5, red, 5);

        cv::line( img, cv::Point(img.cols/2, 0), cv::Point(img.cols/2, img.rows), red, 4); //middle of image

        cv::circle(img, m_robot_points.left_wheel.center2D, 5, red, 5);
        cv::circle(img, m_robot_points.right_wheel.center2D, 5, red, 5);

        cv::namedWindow(parameters.robot+"_TRACKER", CV_WINDOW_NORMAL);
        cv::imshow(parameters.robot+"_TRACKER", img);
        cv::waitKey(10);
    }
}

}
}
}

/** @} */
