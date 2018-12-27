/**
 * @addtogroup DFNs
 * @{
 */

#include "DisparityImage.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>

namespace CDFF
{
namespace DFN
{
namespace DisparityImage
{

DisparityImage::DisparityImage()
{
    parametersHelper.AddParameter<int>("DisparityImageParams", "minDisparity", parameters.minDisparity, DEFAULT_PARAMETERS.minDisparity);
    parametersHelper.AddParameter<int>("DisparityImageParams", "numDisparities", parameters.numDisparities, DEFAULT_PARAMETERS.numDisparities);
    parametersHelper.AddParameter<int>("DisparityImageParams", "blockSize", parameters.blockSize, DEFAULT_PARAMETERS.blockSize);
    parametersHelper.AddParameter<int>("DisparityImageParams", "P1", parameters.P1, DEFAULT_PARAMETERS.P1);
    parametersHelper.AddParameter<int>("DisparityImageParams", "P2", parameters.P2, DEFAULT_PARAMETERS.P2);
    parametersHelper.AddParameter<int>("DisparityImageParams", "disp12MaxDiff", parameters.disp12MaxDiff, DEFAULT_PARAMETERS.disp12MaxDiff);
    parametersHelper.AddParameter<int>("DisparityImageParams", "preFilterCap", parameters.preFilterCap, DEFAULT_PARAMETERS.preFilterCap);
    parametersHelper.AddParameter<int>("DisparityImageParams", "uniquenessRatio", parameters.uniquenessRatio, DEFAULT_PARAMETERS.uniquenessRatio);
    parametersHelper.AddParameter<int>("DisparityImageParams", "speckleWindowSize", parameters.speckleWindowSize, DEFAULT_PARAMETERS.speckleWindowSize);
    parametersHelper.AddParameter<int>("DisparityImageParams", "speckleRange", parameters.speckleRange, DEFAULT_PARAMETERS.speckleRange);
    parametersHelper.AddParameter<int>("DisparityImageParams", "mode", parameters.mode, DEFAULT_PARAMETERS.mode);

    configurationFilePath = "";
}

DisparityImage::~DisparityImage()
{
}

void DisparityImage::configure()
{
    if (configurationFilePath != "")
    {
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void DisparityImage::process()
{
    cv::Mat imgLeft = frameToMat.Convert(&(inFramePair.left));
    cv::Mat imgRight = frameToMat.Convert(&(inFramePair.right));
    cv::Mat disparity;

    cv::Ptr<cv::StereoSGBM> ssgbm = cv::StereoSGBM::create(parameters.minDisparity, parameters.numDisparities, parameters.blockSize, parameters.P1, parameters.P2, parameters.disp12MaxDiff, parameters.preFilterCap, parameters.uniquenessRatio, parameters.speckleWindowSize, parameters.speckleRange, parameters.mode);
    ssgbm->compute(imgLeft, imgRight, disparity);

    // Convert Mat to 
    outRawDisparity.metadata.msgVersion = frame_Version;
    outRawDisparity.metadata = inFramePair.left.metadata;
    outRawDisparity.intrinsic = inFramePair.left.intrinsic;
    outRawDisparity.extrinsic = inFramePair.left.extrinsic;

    outRawDisparity.metadata.mode = asn1Sccmode_GRAY;
    outRawDisparity.metadata.pixelModel = asn1Sccpix_DISP;
    outRawDisparity.metadata.errValues.arr[0].type = asn1Sccerror_UNDEFINED;
    outRawDisparity.metadata.errValues.arr[0].value = -16.0;
    outRawDisparity.metadata.errValues.nCount = 1;

    double minDisp, maxDisp;
    cv::minMaxLoc(disparity, &minDisp, &maxDisp);
    outRawDisparity.metadata.pixelCoeffs.arr[0] = 16.0;
    outRawDisparity.metadata.pixelCoeffs.arr[1] = 0.0;
    outRawDisparity.metadata.pixelCoeffs.arr[2] = inFramePair.baseline;
    outRawDisparity.metadata.pixelCoeffs.arr[3] = maxDisp;
    outRawDisparity.metadata.pixelCoeffs.arr[4] = minDisp;

    outRawDisparity.data.msgVersion = array3D_Version;
    outRawDisparity.data.channels = static_cast<asn1SccT_UInt32>(disparity.channels());;
    outRawDisparity.data.rows = static_cast<asn1SccT_UInt32>(inFramePair.left.data.rows);
    outRawDisparity.data.cols = static_cast<asn1SccT_UInt32>(inFramePair.left.data.cols);
    outRawDisparity.data.depth = static_cast<asn1SccArray3D_depth_t>(disparity.depth());
    outRawDisparity.data.rowSize = disparity.step[0];
    outRawDisparity.data.data.nCount =  static_cast<int>(outRawDisparity.data.rows * outRawDisparity.data.rowSize);
    memcpy(outRawDisparity.data.data.arr, disparity.data, static_cast<size_t>(outRawDisparity.data.data.nCount));
}

void DisparityImage::ValidateParameters()
{
    ASSERT(parameters.minDisparity>=0, "minDisparity has to be positive");
    ASSERT(parameters.numDisparities>0, "numDisparities has to be greater than 0");
    ASSERT(parameters.blockSize%2!=0, "blockSize must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range");
}

const DisparityImage::DisparityImageParams DisparityImage::DEFAULT_PARAMETERS = {
    .minDisparity = 0,
    .numDisparities = 16,
    .blockSize = 3,
    .P1 = 0,
    .P2 = 0,
    .disp12MaxDiff = 0,
    .preFilterCap = 0,
    .uniquenessRatio = 0,
    .speckleWindowSize = 0,
    .speckleRange = 0,
    .mode = cv::StereoSGBM::MODE_SGBM};
}
}
}

/** @} */
