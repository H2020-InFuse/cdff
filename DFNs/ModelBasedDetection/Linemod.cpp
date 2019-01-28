/**
 * @addtogroup DFNs
 * @{
 */

#include <Types/CPP/Pose.hpp>
#include "Linemod.hpp"


namespace CDFF
{
namespace DFN
{
namespace ModelBasedDetection
{

namespace //anonymous namespace
{
void cross(const double a[3], const double b[3], double c[3])
{
  double Cx = a[1] * b[2] - a[2] * b[1];
  double Cy = a[2] * b[0] - a[0] * b[2];
  double Cz = a[0] * b[1] - a[1] * b[0];
  c[0] = Cx; c[1] = Cy; c[2] = Cz;
}

cv::Matx14d normalize(cv::Matx14d& vec)
{
    double norm = cv::norm(vec);
    return cv::Matx14d(vec(0) / norm, vec(1) / norm, vec(2) / norm);
}

void matrix4x4MultiplyPoint(double elem[16], double in[4], double out[4])
{
  double v1 = in[0];
  double v2 = in[1];
  double v3 = in[2];
  double v4 = in[3];

  out[0] = v1*elem[0]  + v2*elem[1]  + v3*elem[2]  + v4*elem[3];
  out[1] = v1*elem[4]  + v2*elem[5]  + v3*elem[6]  + v4*elem[7];
  out[2] = v1*elem[8]  + v2*elem[9]  + v3*elem[10] + v4*elem[11];
  out[3] = v1*elem[12] + v2*elem[13] + v3*elem[14] + v4*elem[15];
}
}

Linemod::Linemod() :
    parametersHelper(), linemodDetector(), retDetection(false), similarity(0.0f), detection(),
    class_id(), template_id(0), detection_R(), detection_T(), cameraPose(), matToPose3D()
{
    parameters = DEFAULT_PARAMETERS;

    parametersHelper.AddParameter<int>("LinemodParams", "T_level0", parameters.T_level0, DEFAULT_PARAMETERS.T_level0);
    parametersHelper.AddParameter<int>("LinemodParams", "T_level1", parameters.T_level1, DEFAULT_PARAMETERS.T_level1);
    parametersHelper.AddParameter<float>("LinemodParams", "matchingThreshold", parameters.matchingThreshold, DEFAULT_PARAMETERS.matchingThreshold);
    parametersHelper.AddParameter<std::string>("LinemodParams", "CAD object name", parameters.cadObjectName, DEFAULT_PARAMETERS.cadObjectName);
    parametersHelper.AddParameter<bool>("LinemodParams", "Use depth", parameters.useDepthModality, DEFAULT_PARAMETERS.useDepthModality);
    parametersHelper.AddParameter<bool>("LinemodParams", "Resize to VGA resolution", parameters.resizeVGA, DEFAULT_PARAMETERS.resizeVGA);

    configurationFilePath = "";
}

Linemod::~Linemod()
{
}

void Linemod::configure()
{
    if (!configurationFilePath.empty()) {
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();

    if (parameters.useDepthModality)
        linemodDetector.InitAs3D(parameters.T_level0, parameters.T_level1);
    else
        linemodDetector.InitAs2D(parameters.T_level0, parameters.T_level1);

    linemodDetector.LoadTraining(parameters.cadObjectName);
}

void Linemod::process()
{
    cv::Mat cv_img_raw = cv::Mat(inimage.data.rows, inimage.data.cols, CV_MAKETYPE((int)(inimage.data.depth), inimage.data.channels), inimage.data.data.arr, inimage.data.rowSize);
    cv::Mat cv_img;
    if (cv_img_raw.type() == CV_8UC1)
        cv::cvtColor(cv_img_raw, cv_img, cv::COLOR_GRAY2BGR);
    else
        cv_img = cv_img_raw;

    if (parameters.resizeVGA)
        cv::resize(cv_img, cv_img, cv::Size(640, 480));

    std::vector<cv::Mat> sources;
    sources.push_back(cv_img);
    if (parameters.useDepthModality)
    {
        cv::Mat cv_depth = cv::Mat(indepth.data.rows, indepth.data.cols, CV_MAKETYPE((int)(indepth.data.depth), indepth.data.channels), indepth.data.data.arr, indepth.data.rowSize);

        if (parameters.resizeVGA)
            cv::resize(cv_depth, cv_depth, cv::Size(640, 480));

        sources.push_back(cv_depth);
    }

    // Perform matching
    cv::linemod::Match matches;
    cv::Rect x_bb;
    cv::Vec3d vec_R, vec_T;

    retDetection = linemodDetector.Detect(sources, static_cast<float>(parameters.matchingThreshold), matches, x_bb, vec_R, vec_T);

    if(retDetection)
    {
        similarity = matches.similarity;
        detection.x = matches.x;
        detection.y = matches.y;
        detection.width = x_bb.width;
        detection.height = x_bb.height;
        class_id = matches.class_id;
        template_id = matches.template_id;
        detection_R = vec_R;
        detection_T = vec_T;
        cameraPose = convertToCameraPose(detection_R, detection_T);
    }

    // Write data to output port
    outSuccess = retDetection;
    if (outSuccess)
    {
        //Camera pose
        PoseWrapper::Pose3DConstPtr tmp = matToPose3D.Convert(cameraPose);
        PoseWrapper::Copy(*tmp, outCamera);
        delete tmp;

        //Detection bounding box
        outDetectionBoundingBox.arr[0].arr[0] = detection.x;
        outDetectionBoundingBox.arr[0].arr[1] = detection.y;
        outDetectionBoundingBox.arr[1].arr[0] = detection.width;
        outDetectionBoundingBox.arr[1].arr[1] = detection.height;
    }
}

bool Linemod::getDetection(float& similarity_, cv::Rect& detection_, std::string& class_id_, int& template_id_,
                                 cv::Vec3d& vec_R_, cv::Vec3d& vec_T_, cv::Mat& cameraPose_)
{
    if (!retDetection)
        return retDetection;

    similarity_ = similarity;
    detection_ = detection;
    class_id_ = class_id;
    template_id_ = template_id;
    vec_R_ = detection_R;
    vec_T_ = detection_T;
    cameraPose_ = cameraPose;

    return retDetection;
}

cv::Mat Linemod::convertToCameraPose(const cv::Vec3d& vec_R, const cv::Vec3d& vec_T)
{
    const cv::Matx13d position(vec_T(0), vec_T(1), vec_T(2));
    const cv::Matx13d focalPoint(0.0, 0.0, 0.0);
    cv::Mat R;
    cv::Rodrigues(vec_R, R);
    const cv::Matx13d viewUp(R.at<double>(0,0), R.at<double>(1,0), R.at<double>(2,0));

    cv::Matx44d matrix = cv::Matx44d::eye();

    // the view directions correspond to the rows of the rotation matrix,
    // so we'll make the connection explicit
    cv::Matx14d viewSideways = matrix.row(0);
    cv::Matx14d orthoViewUp = matrix.row(1);
    cv::Matx14d viewPlaneNormal = matrix.row(2);

    // set the view plane normal from the view vector
    viewPlaneNormal(0) = position(0) - focalPoint(0);
    viewPlaneNormal(1) = position(1) - focalPoint(1);
    viewPlaneNormal(2) = position(2) - focalPoint(2);
    viewPlaneNormal = normalize(viewPlaneNormal);

    // orthogonalize viewUp and compute viewSideways
    cross(viewUp.val, viewPlaneNormal.val, viewSideways.val);
    viewSideways = normalize(viewSideways);
    cross(viewPlaneNormal.val, viewSideways.val, orthoViewUp.val);

    // translate by the vector from the position to the origin
    double delta[4];
    delta[0] = -position(0);
    delta[1] = -position(1);
    delta[2] = -position(2);
    delta[3] = 0.0; // yes, this should be zero, not one

    matrix(0,0) = viewSideways(0);    matrix(0,1) = viewSideways(1);    matrix(0,2) = viewSideways(2);
    matrix(1,0) = orthoViewUp(0);     matrix(1,1) = orthoViewUp(1);     matrix(1,2) = orthoViewUp(2);
    matrix(2,0) = viewPlaneNormal(0); matrix(2,1) = viewPlaneNormal(1); matrix(2,2) = viewPlaneNormal(2);

    matrix4x4MultiplyPoint(matrix.val, delta, delta);

    matrix(0,3) = delta[0];
    matrix(1,3) = delta[1];
    matrix(2,3) = delta[2];

    // return the transformation using our camera frame convention:
    // z /
    //  o-----> x
    //  |
    //  v  y
    cv::Mat gl2cv = (cv::Mat_<double>(4,4) <<  1, 0, 0, 0,
                                               0,-1, 0, 0,
                                               0, 0,-1, 0,
                                               0, 0, 0, 1);

    // To take into account the detection bounding box offset,
    // the correct way should be to do an ICP to refine the pose
    // starting from the pose used during the training
    cv::Mat object2gl(matrix);
    return gl2cv*object2gl;
}

void Linemod::ValidateParameters()
{
    ASSERT(parameters.matchingThreshold >= 0 && parameters.matchingThreshold <= 100, "Matching threshold must be between [0 - 100]");
    ASSERT(!parameters.cadObjectName.empty(), "Path to CAD object name for training data cannot be empty.");
}

const Linemod::LinemodParams Linemod::DEFAULT_PARAMETERS = {
    //Default parameters for DLR servicer
    .T_level0 = 5,
    .T_level1 = 8,
    .matchingThreshold = 80.0f,
    .cadObjectName = "",
    .useDepthModality = false,
    .resizeVGA = false
};

}
}
}
/** @} */
