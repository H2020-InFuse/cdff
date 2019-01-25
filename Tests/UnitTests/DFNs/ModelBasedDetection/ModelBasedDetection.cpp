/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ModelBasedDetection.cpp
 * @date 08/01/2019
 * @author Souriya Trinh
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN Model-based detection.
 *
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <catch.hpp>
#include <Types/CPP/Pose.hpp>
#include <Eigen/Geometry>
#include <ModelBasedDetection/Linemod.hpp>
#include <opencv2/imgcodecs.hpp>

static void cvMat2ASN1(const cv::Mat& mat, asn1SccFrame * asn)
{
    asn1SccArray3D &imageOnly = asn->data;
    imageOnly.msgVersion = array3D_Version;
    imageOnly.rows = static_cast<asn1SccT_UInt32>(mat.rows);
    imageOnly.cols = static_cast<asn1SccT_UInt32>(mat.cols);
    imageOnly.channels = static_cast<asn1SccT_UInt32>(mat.channels());
    imageOnly.depth = static_cast<asn1SccArray3D_depth_t>(mat.depth());
    imageOnly.rowSize = mat.step[0];
    imageOnly.data.nCount = static_cast<int>(imageOnly.rows * imageOnly.rowSize);
    memcpy(imageOnly.data.arr, mat.data, static_cast<size_t>(imageOnly.data.nCount));
}

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (Linemod detection color image)", "[process]" )
{
    //Detect on an image generated during the training using intrinsic parameters
    //that correspond to the DLR cameras
    int index = 60;

    // Load color and depth image
    cv::Mat cvColorImage = cv::imread(cv::format("../tests/Data/Images/Linemod/mod1/Linemod_color_%04d.png", index));

    // Convert to ASN1 format
    asn1SccFrame *colorImage = new asn1SccFrame;
    asn1SccFrame_Initialize(colorImage);

    // Copy image data
    cvMat2ASN1(cvColorImage, colorImage);

    // Instantiate DFN
    CDFF::DFN::ModelBasedDetection::Linemod *linemod = new CDFF::DFN::ModelBasedDetection::Linemod;

    // Configure
    linemod->parameters.useDepthModality = false;
    linemod->parameters.cadObjectName = "../tests/Data/Images/Linemod/mod1/oos";
    linemod->configure();

    // Send input data to DFN
    linemod->imageInput(*colorImage);

    // Run DFN
    linemod->process();

    // Get results
    float similarity = 0.0;
    cv::Rect detection;
    std::string class_id;
    int template_id = -1;
    cv::Vec3d vec_R, vec_T;
    cv::Mat cameraPose;
    bool retDetection = linemod->getDetection(similarity, detection, class_id, template_id, vec_R, vec_T, cameraPose);

    REQUIRE(retDetection);
    REQUIRE(similarity == 100.0f);
    REQUIRE(template_id == index);

    // Query output data from DFN
    const PoseWrapper::Pose3D& camera = linemod->cameraOutput();
    bool success = linemod->successOutput();

    REQUIRE(success);
    Eigen::Quaternionf q(camera.orient.arr[3], camera.orient.arr[0], camera.orient.arr[1], camera.orient.arr[2]);
    Eigen::Matrix3f R = q.toRotationMatrix();
    for (int i = 0; i < 3; i++)
    {
        REQUIRE(camera.pos.arr[i] == Approx(cameraPose.at<double>(i,3)).epsilon(std::numeric_limits<float>::epsilon()));
        for (int j = 0; j < 3; j++)
        {
            REQUIRE(R(i,j) == Approx(cameraPose.at<double>(i,j)).epsilon(1e-5));
        }
    }

    // Cleanup
    delete colorImage;
    delete linemod;
}

TEST_CASE( "Call to process (Linemod detection color + depth images)", "[process]" )
{
    //Detect on image + depth generated during the training using intrinsic parameters
    //that correspond to the DLR cameras
    int index = 509;

    // Load color and depth image
    cv::Mat cvColorImage = cv::imread(cv::format("../tests/Data/Images/Linemod/mod2/Linemod_color_%04d.png", index));
    cv::Mat cvDepthImageRaw = cv::imread(cv::format("../tests/Data/Images/Linemod/mod2/Linemod_depth_%04d.exr", index), cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
    cv::Mat cvDepthImage;
    cvDepthImageRaw.convertTo(cvDepthImage, CV_16U);

    // Convert to ASN1 format
    asn1SccFrame *colorImage = new asn1SccFrame;
    asn1SccFrame *depthImage = new asn1SccFrame;
    asn1SccFrame_Initialize(colorImage);
    asn1SccFrame_Initialize(depthImage);

    // Copy image data
    cvMat2ASN1(cvColorImage, colorImage);
    cvMat2ASN1(cvDepthImage, depthImage);

    // Instantiate DFN
    CDFF::DFN::ModelBasedDetection::Linemod *linemod = new CDFF::DFN::ModelBasedDetection::Linemod;

    // Configure
    linemod->parameters.useDepthModality = true;
    linemod->parameters.cadObjectName = "../tests/Data/Images/Linemod/mod2/oos";
    linemod->configure();

    // Send input data to DFN
    linemod->imageInput(*colorImage);
    linemod->depthInput(*depthImage);

    // Run DFN
    linemod->process();

    // Get results
    float similarity = 0.0;
    cv::Rect detection;
    std::string class_id;
    int template_id = -1;
    cv::Vec3d vec_R, vec_T;
    cv::Mat cameraPose;
    bool retDetection = linemod->getDetection(similarity, detection, class_id, template_id, vec_R, vec_T, cameraPose);

    REQUIRE(retDetection);
    REQUIRE(similarity == 100.0f);
    REQUIRE(template_id == index);

    // Query output data from DFN
    const PoseWrapper::Pose3D& camera = linemod->cameraOutput();
    bool success = linemod->successOutput();

    REQUIRE(success);
    Eigen::Quaternionf q(camera.orient.arr[3], camera.orient.arr[0], camera.orient.arr[1], camera.orient.arr[2]);
    Eigen::Matrix3f R = q.toRotationMatrix();
    for (int i = 0; i < 3; i++)
    {
        REQUIRE(camera.pos.arr[i] == Approx(cameraPose.at<double>(i,3)).epsilon(std::numeric_limits<float>::epsilon()));
        for (int j = 0; j < 3; j++)
        {
            REQUIRE(R(i,j) == Approx(cameraPose.at<double>(i,j)).epsilon(1e-5));
        }
    }

    // Cleanup
    delete colorImage;
    delete depthImage;
    delete linemod;
}

TEST_CASE( "Call to process (Linemod detection color + depth images Blender)", "[process]" )
{
    //Detect on image + depth generated by Blender using a camera pose generated during the training
    //Training is done using intrinsic parameters for Blender camera
    int index = 530;

    // Load color and depth image
    cv::Mat cvColorImage = cv::imread(cv::format("../tests/Data/Images/Linemod/mod2_Blender/Linemod_color_%04d.png", index));
    cv::Mat cvDepthImageRaw = cv::imread(cv::format("../tests/Data/Images/Linemod/mod2_Blender/Linemod_depth_%04d.exr", index), cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
    cv::Mat cvDepthImage;
    cvDepthImageRaw.convertTo(cvDepthImage, CV_16U);

    // Convert to ASN1 format
    asn1SccFrame *colorImage = new asn1SccFrame;
    asn1SccFrame *depthImage = new asn1SccFrame;
    asn1SccFrame_Initialize(colorImage);
    asn1SccFrame_Initialize(depthImage);

    // Copy image data
    cvMat2ASN1(cvColorImage, colorImage);
    cvMat2ASN1(cvDepthImage, depthImage);

    // Instantiate DFN
    CDFF::DFN::ModelBasedDetection::Linemod *linemod = new CDFF::DFN::ModelBasedDetection::Linemod;

    // Configure
    linemod->parameters.useDepthModality = true;
    linemod->parameters.cadObjectName = "../tests/Data/Images/Linemod/mod2_Blender/oos";
    linemod->configure();

    // Send input data to DFN
    linemod->imageInput(*colorImage);
    linemod->depthInput(*depthImage);

    // Run DFN
    linemod->process();

    // Get results
    float similarity = 0.0;
    cv::Rect detection;
    std::string class_id;
    int template_id = -1;
    cv::Vec3d vec_R, vec_T;
    cv::Mat cameraPose;
    bool retDetection = linemod->getDetection(similarity, detection, class_id, template_id, vec_R, vec_T, cameraPose);

    REQUIRE(retDetection);
    REQUIRE(similarity >= 99.0f);
    REQUIRE(template_id == index);

    // Query output data from DFN
    const PoseWrapper::Pose3D& camera = linemod->cameraOutput();
    bool success = linemod->successOutput();

    REQUIRE(success);
    Eigen::Quaternionf q(camera.orient.arr[3], camera.orient.arr[0], camera.orient.arr[1], camera.orient.arr[2]);
    Eigen::Matrix3f R = q.toRotationMatrix();
    for (int i = 0; i < 3; i++)
    {
        REQUIRE(camera.pos.arr[i] == Approx(cameraPose.at<double>(i,3)).epsilon(std::numeric_limits<float>::epsilon()));
        for (int j = 0; j < 3; j++)
        {
            REQUIRE(R(i,j) == Approx(cameraPose.at<double>(i,j)).epsilon(1e-5));
        }
    }

    // Cleanup
    delete colorImage;
    delete depthImage;
    delete linemod;
}

/** @} */
