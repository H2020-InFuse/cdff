/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file LinemodDetect.cpp
 * @date 08/01/2019
 * @author Souriya Trinh
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN Linemod detection.
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
#include <LinemodDetect/LinemodDetect.hpp>
#include <opencv2/imgcodecs.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (Linemod detection color image)", "[process]" )
{
    int index = 60;

    // Load color and depth image
    cv::Mat cvColorImage = cv::imread(cv::format("../tests/Data/Images/Linemod/mod1/Linemod_color_%04d.png", index));

    // Convert to ASN1 format
    asn1SccFrame *colorImage = new asn1SccFrame;
    asn1SccFrame_Initialize(colorImage);

    // Copy image data
    asn1SccArray3D &imageOnly = colorImage->data;
    imageOnly.msgVersion = array3D_Version;
    imageOnly.rows = static_cast<asn1SccT_UInt32>(cvColorImage.rows);
    imageOnly.cols = static_cast<asn1SccT_UInt32>(cvColorImage.cols);
    imageOnly.channels = static_cast<asn1SccT_UInt32>(cvColorImage.channels());
    imageOnly.depth = static_cast<asn1SccArray3D_depth_t>(cvColorImage.depth());
    imageOnly.rowSize = cvColorImage.step[0];
    imageOnly.data.nCount = static_cast<int>(imageOnly.rows * imageOnly.rowSize);
    memcpy(imageOnly.data.arr, cvColorImage.data, static_cast<size_t>(imageOnly.data.nCount));

    // Instantiate DFN
    CDFF::DFN::LinemodDetect::LinemodDetect *linemodDetect = new CDFF::DFN::LinemodDetect::LinemodDetect;

    // Configure
    linemodDetect->parameters.useDepthModality = false;
    linemodDetect->parameters.cadObjectName = "../tests/Data/Images/Linemod/mod1/oos";
    linemodDetect->configure();

    // Send input data to DFN
    linemodDetect->imageInput(*colorImage);

    // Run DFN
    linemodDetect->process();

    // Get results
    float similarity = 0.0;
    cv::Rect detection;
    std::string class_id;
    int template_id = -1;
    cv::Vec3d vec_R, vec_T;
    cv::Mat cameraPose;
    bool retDetection = linemodDetect->getDetection(similarity, detection, class_id, template_id, vec_R, vec_T, cameraPose);

    REQUIRE(retDetection);
    REQUIRE(similarity == 100.0f);
    REQUIRE(template_id == index);

    // Query output data from DFN
    const PoseWrapper::Pose3D& camera = linemodDetect->cameraOutput();
    bool success = linemodDetect->successOutput();

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
    delete linemodDetect;
}

TEST_CASE( "Call to process (Linemod detection color + depth images)", "[process]" )
{
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
    asn1SccArray3D &imageOnly = colorImage->data;
    imageOnly.msgVersion = array3D_Version;
    imageOnly.rows = static_cast<asn1SccT_UInt32>(cvColorImage.rows);
    imageOnly.cols = static_cast<asn1SccT_UInt32>(cvColorImage.cols);
    imageOnly.channels = static_cast<asn1SccT_UInt32>(cvColorImage.channels());
    imageOnly.depth = static_cast<asn1SccArray3D_depth_t>(cvColorImage.depth());
    imageOnly.rowSize = cvColorImage.step[0];
    imageOnly.data.nCount = static_cast<int>(imageOnly.rows * imageOnly.rowSize);
    memcpy(imageOnly.data.arr, cvColorImage.data, static_cast<size_t>(imageOnly.data.nCount));

    asn1SccArray3D &depthOnly = depthImage->data;
    depthOnly.msgVersion = array3D_Version;
    depthOnly.rows = static_cast<asn1SccT_UInt32>(cvDepthImage.rows);
    depthOnly.cols = static_cast<asn1SccT_UInt32>(cvDepthImage.cols);
    depthOnly.channels = static_cast<asn1SccT_UInt32>(cvDepthImage.channels());
    depthOnly.depth = static_cast<asn1SccArray3D_depth_t>(cvDepthImage.depth());
    depthOnly.rowSize = cvDepthImage.step[0];
    depthOnly.data.nCount = static_cast<int>(depthOnly.rows * depthOnly.rowSize);
    memcpy(depthOnly.data.arr, cvDepthImage.data, static_cast<size_t>(depthOnly.data.nCount));

    // Instantiate DFN
    CDFF::DFN::LinemodDetect::LinemodDetect *linemodDetect = new CDFF::DFN::LinemodDetect::LinemodDetect;

    // Configure
    linemodDetect->parameters.useDepthModality = true;
    linemodDetect->parameters.cadObjectName = "../tests/Data/Images/Linemod/mod2/oos";
    linemodDetect->configure();

    // Send input data to DFN
    linemodDetect->imageInput(*colorImage);
    linemodDetect->depthInput(*depthImage);

    // Run DFN
    linemodDetect->process();

    // Get results
    float similarity = 0.0;
    cv::Rect detection;
    std::string class_id;
    int template_id = -1;
    cv::Vec3d vec_R, vec_T;
    cv::Mat cameraPose;
    bool retDetection = linemodDetect->getDetection(similarity, detection, class_id, template_id, vec_R, vec_T, cameraPose);

    REQUIRE(retDetection);
    REQUIRE(similarity == 100.0f);
    REQUIRE(template_id == index);

    // Query output data from DFN
    const PoseWrapper::Pose3D& camera = linemodDetect->cameraOutput();
    bool success = linemodDetect->successOutput();

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
    delete linemodDetect;
}

/** @} */
