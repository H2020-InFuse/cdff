/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StereoSlamorb.cpp
 * @author Vincent Bissonnette
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN StereoSlamOrb.
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
#include <StereoSlam/StereoSlamOrb.hpp>
#include <Types/C/Frame.h>
#include <Types/C/TransformWithCovariance.h>
#include <opencv2/highgui/highgui.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
/**
 * @brief loadFramePair Creates a framePair from 2 image files
 * @param pathLeft Path to left image file
 * @param pathRight Path to right image file
 * @param framepair Output FramePair
 */
void loadFramePair(const std::string pathLeft, const std::string pathRight, asn1SccFramePair& framePair)
{
    // Loads two grayscaled rectified images
    cv::Mat cvLeftImage = cv::imread(pathLeft.c_str(), cv::IMREAD_GRAYSCALE);
    cv::Mat cvRightImage = cv::imread(pathRight.c_str(), cv::IMREAD_GRAYSCALE);

    // Initialise the frame pair and set its metadata
    asn1SccFramePair_Initialize(&framePair);
    framePair.msgVersion = frame_Version;
    framePair.baseline = 0.270268442641143;

    // Fill the left image data
    {
        framePair.left.msgVersion = frame_Version;
        framePair.left.metadata.msgVersion = frame_Version;
        framePair.left.metadata.status = asn1Sccstatus_VALID;
        framePair.left.metadata.pixelModel = asn1Sccpix_UNDEF;
        framePair.left.metadata.mode = asn1Sccmode_GRAY;

        framePair.left.intrinsic.msgVersion = frame_Version;
        framePair.left.intrinsic.cameraModel = asn1Scccam_PINHOLE;
        framePair.left.intrinsic.cameraMatrix.arr[0].arr[0] = 1018.6;
        framePair.left.intrinsic.cameraMatrix.arr[0].arr[1] = 0;
        framePair.left.intrinsic.cameraMatrix.arr[0].arr[2] = 956.501;
        framePair.left.intrinsic.cameraMatrix.arr[1].arr[0] = 0;
        framePair.left.intrinsic.cameraMatrix.arr[1].arr[1] = 1018.6;
        framePair.left.intrinsic.cameraMatrix.arr[1].arr[2] = 617.314;
        framePair.left.intrinsic.cameraMatrix.arr[2].arr[0] = 0;
        framePair.left.intrinsic.cameraMatrix.arr[2].arr[1] = 0;
        framePair.left.intrinsic.cameraMatrix.arr[2].arr[2] = 1;

        asn1SccArray3D &imageOnly = framePair.left.data;
        imageOnly.msgVersion = array3D_Version;
        imageOnly.rows = static_cast<asn1SccT_UInt32>(cvLeftImage.rows);
        imageOnly.cols = static_cast<asn1SccT_UInt32>(cvLeftImage.cols);
        imageOnly.channels = static_cast<asn1SccT_UInt32>(cvLeftImage.channels());
        imageOnly.depth = static_cast<asn1SccArray3D_depth_t>(cvLeftImage.depth());
        imageOnly.rowSize = cvLeftImage.step[0];
        imageOnly.data.nCount = static_cast<int>(imageOnly.rows * imageOnly.rowSize);
        memcpy(imageOnly.data.arr, cvLeftImage.data, static_cast<size_t>(imageOnly.data.nCount));
    }

    // Fill the right image data
    {
        framePair.right.msgVersion = frame_Version;
        framePair.right.metadata.msgVersion = frame_Version;
        framePair.right.metadata.status = asn1Sccstatus_VALID;
        framePair.right.metadata.pixelModel = asn1Sccpix_UNDEF;
        framePair.right.metadata.mode = asn1Sccmode_GRAY;

        framePair.right.intrinsic.msgVersion = frame_Version;
        framePair.right.intrinsic.cameraModel = asn1Scccam_PINHOLE;
        framePair.right.intrinsic.cameraMatrix.arr[0].arr[0] = 1018.6;
        framePair.right.intrinsic.cameraMatrix.arr[0].arr[1] = 0;
        framePair.right.intrinsic.cameraMatrix.arr[0].arr[2] = 956.501;
        framePair.right.intrinsic.cameraMatrix.arr[1].arr[0] = 0;
        framePair.right.intrinsic.cameraMatrix.arr[1].arr[1] = 1018.6;
        framePair.right.intrinsic.cameraMatrix.arr[1].arr[2] = 617.314;
        framePair.right.intrinsic.cameraMatrix.arr[2].arr[0] = 0;
        framePair.right.intrinsic.cameraMatrix.arr[2].arr[1] = 0;
        framePair.right.intrinsic.cameraMatrix.arr[2].arr[2] = 1;
        asn1SccArray3D &imageOnly = framePair.right.data;
        imageOnly.msgVersion = array3D_Version;
        imageOnly.rows = static_cast<asn1SccT_UInt32>(cvRightImage.rows);
        imageOnly.cols = static_cast<asn1SccT_UInt32>(cvRightImage.cols);
        imageOnly.channels = static_cast<asn1SccT_UInt32>(cvRightImage.channels());
        imageOnly.depth = static_cast<asn1SccArray3D_depth_t>(cvRightImage.depth());
        imageOnly.rowSize = cvRightImage.step[0];
        imageOnly.data.nCount = static_cast<int>(imageOnly.rows * imageOnly.rowSize);
        memcpy(imageOnly.data.arr, cvRightImage.data, static_cast<size_t>(imageOnly.data.nCount));
    }
}

TEST_CASE( "Call to process (StereoSlamOrb)", "[process]" )
{
    // Load two grayscale rectified image pairs
    asn1SccFramePair* pair1 = new asn1SccFramePair();
    asn1SccFramePair* pair2 = new asn1SccFramePair();

    loadFramePair("../tests/Data/Images/MinnieStereo/MinnieRectLeft.png", "../tests/Data/Images/MinnieStereo/MinnieRectRight.png", *pair1);
    loadFramePair("../tests/Data/Images/MinnieStereo/MinnieRectLeftNext.png", "../tests/Data/Images/MinnieStereo/MinnieRectRightNext.png", *pair2);

    // Instantiate DFN
    CDFF::DFN::StereoSlam::StereoSlamOrb *slam = new CDFF::DFN::StereoSlam::StereoSlamOrb();

    // Configure DFN
    slam->setConfigurationFile("../tests/ConfigurationFiles/DFNs/StereoSlam/StereoSlamOrb.yaml");

    slam->configure();

    //Send input data to DFN
    slam->framePairInput(*pair1);

    // Run DFN once to initialize the slam system
    slam->process();

    // Send a second frame to track
    slam->framePairInput(*pair2);

    // Process a second time
    slam->process();

    // Query output data from DFN
    const asn1SccTransformWithCovariance& output = slam->PoseOutput();

    REQUIRE( output.metadata.msgVersion == transformWithCovariance_version );
    // Displacement between 2 test images is currently ~12cms
    REQUIRE( sqrt(output.data.translation.arr[0]*output.data.translation.arr[0]
            + output.data.translation.arr[1]*output.data.translation.arr[1]
            + output.data.translation.arr[2]*output.data.translation.arr[2]) > 0.0 );

    // Cleanup
    delete(slam);
    delete(pair1);
    delete(pair2);

}


/** @} */
