/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFPCs
 * @{
 */

#include "EdgeModelContourMatching.hpp"
#include <Errors/Assert.hpp>
#include <Types/C/RigidBodyState.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>  // std::cout std::endl
#include <string>    // std::string std::memcpy
#include <algorithm> // std::max
#include <cstdio>    // std::sprintf
#include <cstdlib>   // EXIT_SUCCESS EXIT_FAILURE

using namespace DLRtracker;

namespace CDFF
{
namespace DFPC
{
namespace ModelBasedVisualTracking
{

EdgeModelContourMatching::EdgeModelContourMatching() :
     parser(),
     DLRTracker(),
     status(-1),
     numberOfCameras(1),
     images(),
     imageOutputColor(NULL),
     xResolutionMax(0),
     yResolutionMax(0),
     frameToMat()
{
    //              Uchar UcharPtr    Double   Float      Int   IntPtr    Short
    doBigMalloc(102400000, 1200000, 12000000, 200000, 5600000, 1400000, 3200000);
    configurationFilePath = "";
}

EdgeModelContourMatching::~EdgeModelContourMatching()
{
    doBigFree();
}

void EdgeModelContourMatching::allocateImageMemory()
{
    numberOfCameras = DLRTracker.getNcams();
    for (int c = 0; c < numberOfCameras; c++)
    {
        images[c] = myMallocUchar(DLRTracker.getXres(c) * DLRTracker.getYres(c) * sizeof(unsigned char));
        xResolutionMax = std::max(DLRTracker.getXres(c), xResolutionMax);
        yResolutionMax = std::max(DLRTracker.getYres(c), yResolutionMax);
    }
    imageOutputColor = myMallocUchar(3 * xResolutionMax * yResolutionMax * sizeof(unsigned char));
}

void EdgeModelContourMatching::run()
{
    ASSERT(numberOfCameras < 3, "Too many camera: maximum is two");
    for (int c = 0; c < numberOfCameras; c++)
    {
        if (c == 0)
        {
            cv::Mat inputLeftImage = frameToMat.Convert(&inImageLeft);
            ASSERT(inputLeftImage.channels() == 1, "Unsupported image type: tracker input must be a grayscale image");
            ASSERT(inputLeftImage.rows > 0 && inputLeftImage.cols > 0,"Empty image: tracker input must be a grayscale image");
            std::memcpy(images[c], inputLeftImage.data, DLRTracker.getXres(c)*DLRTracker.getYres(c)*sizeof(unsigned char));
        }
        if (numberOfCameras > 1 && c == 1)
        {
            cv::Mat inputRightImage = frameToMat.Convert(&inImageRight);
            ASSERT(inputRightImage.channels() == 1, "Unsupported image type: tracker input must be a grayscale image");
            ASSERT(inputRightImage.rows > 0 && inputRightImage.cols > 0, "Unsupported image type: tracker input must be a grayscale image");
            std::memcpy(images[c], inputRightImage.data, DLRTracker.getXres(c)*DLRTracker.getYres(c)*sizeof(unsigned char));
        }
    }

    double timeImages = inImageTime.microseconds * 0.000001; // timeImages in seconds

    double guessT0[16];
    double velocity0[6];
    double rotTrasl[6];

    double time0 = inInitTime.microseconds * 0.000001; // time0 in seconds

    bool useInitialGuess = inDoInit;

    ConvertASN1StateToState(inInit, rotTrasl, velocity0);
    TfromAngleAxis(rotTrasl, guessT0);

    double egomotion[16];
    ConvertASN1StateToState(inEgoMotion, rotTrasl);
    TfromAngleAxis(rotTrasl, egomotion);

    double estimatedT[16];
    double estimatedVelocity[6];
    double ErrorCovariance[6*6];
    // outputs for ASN: outSuccess, outState
    outSuccess = edgeMatching(images, timeImages, egomotion, guessT0, velocity0,
        time0, useInitialGuess, estimatedT, estimatedVelocity, ErrorCovariance);
    // output: state estimates: estimatedT, estimatedVelocity
    if (outSuccess)
    {
      AngleAxisFromT(estimatedT, rotTrasl);
    }
    outState = ConvertStateToASN1State(rotTrasl, estimatedVelocity);

    // for visualization, uncomment DLRTRACKER_USE_OPENCV_DISPLAY in file
    // common.h of the DLRtracker_core library
#ifdef DLRTRACKER_USE_OPENCV_DISPLAY
    std::string wname;
    int xres = DLRTracker.getXres(0);
    int yres = DLRTracker.getYres(0);
    cv::Mat imageOutputColorCV(xres, yres, CV_8UC3);
    std::memcpy(imageOutputColor, imageOutputColorCV.data, xres*yres);

    for (int c = 0; c < numberOfCameras; c++)
    {
        DLRTracker.drawResult(estimatedT, imageOutputColor, c, true, 100);
        std::sprintf((char*)wname.c_str(), "result%d", c);
        createWindow(wname, 1);
        showImage(wname, imageOutputColor, xres, yres, 3);
    }

    char c = waitKey(10);
    if (c == 27)
    {
        exit(EXIT_SUCCESS);
    }
#endif
}

void EdgeModelContourMatching::setup()
{
    double setup_global_array[_MAX_PARSE_ARRAY];
    int setup_global_array_counter;
    const char* pathToSpecifications = configurationFilePath.c_str();

    std::cout << "Path to configuration file: " << configurationFilePath << std::endl;

    if (parser.parseAllFiles(
            pathToSpecifications, "camera_parameters.txt", "tracker_parameters.txt", "theModel_client.txt",
            setup_global_array_counter, setup_global_array) != 0)
    {
        exit(EXIT_FAILURE);
    }

    if (DLRTracker.setupFromGlobalArray(setup_global_array_counter, setup_global_array) != 0)
    {
        exit(EXIT_FAILURE);
    }

    DLRTracker.getObjectModel().changeLocalFrame(0);
    allocateImageMemory();

    dumpMemoryAlloc();
}

bool EdgeModelContourMatching::edgeMatching(
    unsigned char** images, double timeImages, double* egomotion, double* guessT0,
    double* velocity0, double time0, bool useInitialGuess, double* estimatedT,
    double* estimatedVelocity, double* ErrorCovariance)
{
    double degreesOfFreedom[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    status = DLRTracker.poseEstimation(
        images, timeImages, egomotion, guessT0, velocity0, time0, estimatedT,
        estimatedVelocity, ErrorCovariance, degreesOfFreedom, useInitialGuess,
        false, false, false);

    return status == 0; // success if status == 0
}

void EdgeModelContourMatching::ConvertASN1StateToState(
    asn1SccRigidBodyState& state, double* pose, double* velocity)
{
    pose[0] = state.orient.arr[0];
    pose[1] = state.orient.arr[1];
    pose[2] = state.orient.arr[2];

    pose[3] = state.pos.arr[0];
    pose[4] = state.pos.arr[1];
    pose[5] = state.pos.arr[2];

    if (velocity != NULL)
    {
        velocity[0] = state.angular_velocity.arr[0];
        velocity[1] = state.angular_velocity.arr[1];
        velocity[2] = state.angular_velocity.arr[2];

        velocity[3] = state.velocity.arr[3];
        velocity[4] = state.velocity.arr[4];
        velocity[5] = state.velocity.arr[5];
    }
}

asn1SccRigidBodyState EdgeModelContourMatching::ConvertStateToASN1State(
    double* pose, double* velocity)
{
    asn1SccRigidBodyState ASN1State;

    ASN1State.orient.arr[0] = pose[0];
    ASN1State.orient.arr[1] = pose[1];
    ASN1State.orient.arr[2] = pose[2];

    ASN1State.pos.arr[0] = pose[3];
    ASN1State.pos.arr[1] = pose[4];
    ASN1State.pos.arr[2] = pose[5];

    ASN1State.angular_velocity.arr[0] = velocity[0];
    ASN1State.angular_velocity.arr[1] = velocity[1];
    ASN1State.angular_velocity.arr[2] = velocity[2];

    ASN1State.velocity.arr[0] = velocity[3];
    ASN1State.velocity.arr[1] = velocity[4];
    ASN1State.velocity.arr[2] = velocity[5];

    return ASN1State;
}

}
}
}

/** @} */
