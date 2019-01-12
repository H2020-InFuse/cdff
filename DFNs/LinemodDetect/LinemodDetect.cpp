/**
 * @addtogroup DFNs
 * @{
 */

#include <Types/CPP/Pose.hpp>
#include "LinemodDetect.hpp"
#include <vtkMatrix4x4.h>
#include <vtkMath.h>

namespace CDFF
{
namespace DFN
{
namespace LinemodDetect
{

LinemodDetect::LinemodDetect() :
    parametersHelper(), linemodDetector(), retDetection(false), similarity(0.0f), detection(),
    class_id(), template_id(0), detection_R(), detection_T(), cameraPose(), matToPose3D()
{
    parameters = DEFAULT_PARAMETERS;

    parametersHelper.AddParameter<int>("LinemodDetectParams", "T_level0", parameters.T_level0, DEFAULT_PARAMETERS.T_level0);
    parametersHelper.AddParameter<int>("LinemodDetectParams", "T_level1", parameters.T_level1, DEFAULT_PARAMETERS.T_level1);
    parametersHelper.AddParameter<float>("LinemodDetectParams", "matchingThreshold", parameters.matchingThreshold, DEFAULT_PARAMETERS.matchingThreshold);
    parametersHelper.AddParameter<std::string>("LinemodDetectParams", "CAD object name", parameters.cadObjectName, DEFAULT_PARAMETERS.cadObjectName);
    parametersHelper.AddParameter<bool>("LinemodDetectParams", "Use depth", parameters.useDepthModality, DEFAULT_PARAMETERS.useDepthModality);
    parametersHelper.AddParameter<bool>("LinemodDetectParams", "Resize to VGA resolution", parameters.resizeVGA, DEFAULT_PARAMETERS.resizeVGA);
    parametersHelper.AddParameter<bool>("LinemodDetectParams", "Display Linemod result", parameters.displayResult, DEFAULT_PARAMETERS.displayResult);

    configurationFilePath = "";
}

LinemodDetect::~LinemodDetect()
{
}

void LinemodDetect::configure()
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

void LinemodDetect::process()
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

//        printf("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n",
//               static_cast<double>(matches.similarity), matches.x, matches.y, matches.class_id.c_str(), matches.template_id);
//        printf("Pose computed x = %f, y = %f, z = %f, theta.ux = %f, theta.uy = %f, theta.uz = %f\n",
//               vec_T[0], vec_T[1], vec_T[2], vec_R[0], vec_R[1], vec_R[2]);

        if (parameters.displayResult)
        {
            const int  num_modalities = parameters.useDepthModality ? 2 : 1;

            // Draw matching template
            cv::Mat display = cv_img.clone();
            const std::vector<cv::linemod::Template>& templates = linemodDetector.getTemplates(matches.class_id, matches.template_id);
            linemodDetector.drawResponse(templates, num_modalities, display, cv::Point(matches.x, matches.y), linemodDetector.getT(0));
            cv::rectangle(display,cv::Rect(matches.x, matches.y, x_bb.width, x_bb.height),cv::Scalar(0,255,0),2);

            cv::imshow("Linemod detection result", display);
            cv::waitKey(5);
        }
    }

    // Write data to output port
    outSuccess = retDetection;
    if (outSuccess)
    {
        PoseWrapper::Pose3DConstPtr tmp = matToPose3D.Convert(cameraPose);
        PoseWrapper::Copy(*tmp, outCamera);
        delete tmp;
    }
}

bool LinemodDetect::getDetection(float& similarity_, cv::Rect& detection_, std::string& class_id_, int& template_id_,
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

cv::Mat LinemodDetect::convertToCameraPose(const cv::Vec3d& vec_R, const cv::Vec3d& vec_T)
{
    // From vtkPerspectiveTransform::SetupCamera()
    const double position[3] = {vec_T[0], vec_T[1], vec_T[2]};
    const double focalPoint[3] = {0.0, 0.0, 0.0};
    cv::Mat R;
    cv::Rodrigues(vec_R, R);
    const double viewUp[3] = {R.at<double>(0,0), R.at<double>(1,0), R.at<double>(2,0)};

    double matrix[4][4];
    vtkMatrix4x4::Identity(*matrix);

    // the view directions correspond to the rows of the rotation matrix,
    // so we'll make the connection explicit
    double *viewSideways =    matrix[0];
    double *orthoViewUp =     matrix[1];
    double *viewPlaneNormal = matrix[2];

    // set the view plane normal from the view vector
    viewPlaneNormal[0] = position[0] - focalPoint[0];
    viewPlaneNormal[1] = position[1] - focalPoint[1];
    viewPlaneNormal[2] = position[2] - focalPoint[2];
    vtkMath::Normalize(viewPlaneNormal);

    // orthogonalize viewUp and compute viewSideways
    vtkMath::Cross(viewUp,viewPlaneNormal,viewSideways);
    vtkMath::Normalize(viewSideways);
    vtkMath::Cross(viewPlaneNormal,viewSideways,orthoViewUp);

    // translate by the vector from the position to the origin
    double delta[4];
    delta[0] = -position[0];
    delta[1] = -position[1];
    delta[2] = -position[2];
    delta[3] = 0.0; // yes, this should be zero, not one

    vtkMatrix4x4::MultiplyPoint(*matrix,delta,delta);

    matrix[0][3] = delta[0];
    matrix[1][3] = delta[1];
    matrix[2][3] = delta[2];

    // return the transformation using our camera frame convention:
    // z /
    //  o-----> x
    //  |
    //  v  y
    cv::Mat matT(4,4,CV_64F);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            matT.at<double>(i,j) = matrix[i][j];
        }
    }

    cv::Mat gl2cv = (cv::Mat_<double>(4,4) <<  1, 0, 0, 0,
                                               0,-1, 0, 0,
                                               0, 0,-1, 0,
                                               0, 0, 0, 1);

    // To take into account the detection bounding box offset,
    // the correct way should be to do an ICP to refine the pose
    // starting from the pose used during the training
    return gl2cv*matT;
}

void LinemodDetect::ValidateParameters()
{
    ASSERT(parameters.matchingThreshold >= 0 && parameters.matchingThreshold <= 100, "Matching threshold must be between [0 - 100]");
    ASSERT(!parameters.cadObjectName.empty(), "Path to CAD object name for training data cannot be empty.");
}

const LinemodDetect::LinemodDetectParams LinemodDetect::DEFAULT_PARAMETERS = {
    //Default parameters for DLR servicer
    .T_level0 = 5,
    .T_level1 = 8,
    .matchingThreshold = 80.0f,
    .cadObjectName = "",
    .useDepthModality = false,
    .resizeVGA = false,
    .displayResult = false
};

}
}
}
/** @} */
