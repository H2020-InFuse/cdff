/**
 * @addtogroup DFNs
 * @{
 */

#include "ImagePairDegradation.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace CDFF
{
namespace DFN
{
namespace ImagePairDegradation
{

ImagePairDegradation::ImagePairDegradation()
{
    parametersHelper.AddParameter<int>("ImageDegradationParams", "xratio", parameters.xratio, DEFAULT_PARAMETERS.xratio);
    parametersHelper.AddParameter<int>("ImageDegradationParams", "yratio", parameters.yratio, DEFAULT_PARAMETERS.yratio);
    parametersHelper.AddParameter<int>("ImageDegradationParams", "method", parameters.method, DEFAULT_PARAMETERS.method);

    configurationFilePath = "";
}

ImagePairDegradation::~ImagePairDegradation()
{
}

void ImagePairDegradation::configure()
{
    if(configurationFilePath != ""){
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void ImagePairDegradation::process()
{
    cv::Mat inLeft(static_cast<int>(inOriginalImagePair.left.data.rows), static_cast<int>(inOriginalImagePair.left.data.cols), CV_MAKETYPE(static_cast<int>(inOriginalImagePair.left.data.depth), static_cast<int>(inOriginalImagePair.left.data.channels)), inOriginalImagePair.left.data.data.arr, inOriginalImagePair.left.data.rowSize);
    cv::Mat inRight(static_cast<int>(inOriginalImagePair.right.data.rows), static_cast<int>(inOriginalImagePair.right.data.cols), CV_MAKETYPE(static_cast<int>(inOriginalImagePair.right.data.depth), static_cast<int>(inOriginalImagePair.right.data.channels)), inOriginalImagePair.right.data.data.arr, inOriginalImagePair.right.data.rowSize);
    cv::Mat outLeft, outRight;

    cv::resize(inLeft, outLeft, cv::Size(inLeft.cols / parameters.xratio, inLeft.rows / parameters.yratio), 0, 0, static_cast<cv::InterpolationFlags>(parameters.method));
    cv::resize(inRight, outRight, cv::Size(inRight.cols / parameters.xratio, inRight.rows / parameters.yratio), 0, 0, static_cast<cv::InterpolationFlags>(parameters.method));

    outDegradedImagePair.msgVersion = frame_Version;
    outDegradedImagePair.baseline = inOriginalImagePair.baseline;

    // Getting Left image
    {
        // init the structure
        outDegradedImagePair.left.msgVersion = frame_Version;

        outDegradedImagePair.left.intrinsic = inOriginalImagePair.left.intrinsic;

        // Dividing fx, fy, s, cx, cy by Ratio
        outDegradedImagePair.left.intrinsic.cameraMatrix.arr[0].arr[0] = inOriginalImagePair.left.intrinsic.cameraMatrix.arr[0].arr[0]/parameters.xratio;
        outDegradedImagePair.left.intrinsic.cameraMatrix.arr[0].arr[1] = inOriginalImagePair.left.intrinsic.cameraMatrix.arr[0].arr[1]/parameters.xratio;
        outDegradedImagePair.left.intrinsic.cameraMatrix.arr[0].arr[2] = inOriginalImagePair.left.intrinsic.cameraMatrix.arr[0].arr[2]/parameters.xratio;
        outDegradedImagePair.left.intrinsic.cameraMatrix.arr[1].arr[1] = inOriginalImagePair.left.intrinsic.cameraMatrix.arr[1].arr[1]/parameters.yratio;
        outDegradedImagePair.left.intrinsic.cameraMatrix.arr[1].arr[2] = inOriginalImagePair.left.intrinsic.cameraMatrix.arr[1].arr[2]/parameters.yratio;

        outDegradedImagePair.left.extrinsic = inOriginalImagePair.left.extrinsic;
        outDegradedImagePair.left.metadata = inOriginalImagePair.left.metadata;

        // Array3D
        {
            outDegradedImagePair.left.data.msgVersion = array3D_Version;
            outDegradedImagePair.left.data.rows = static_cast<asn1SccT_UInt32>(outLeft.rows);
            outDegradedImagePair.left.data.cols = static_cast<asn1SccT_UInt32>(outLeft.cols);
            outDegradedImagePair.left.data.channels = static_cast<asn1SccT_UInt32>(outLeft.channels());
            outDegradedImagePair.left.data.depth = static_cast<asn1SccArray3D_depth_t>(outLeft.depth());
            outDegradedImagePair.left.data.rowSize = static_cast<asn1SccT_UInt32>(outLeft.step[0]);
            outDegradedImagePair.left.data.data.nCount = static_cast<int>(outDegradedImagePair.left.data.rows * outDegradedImagePair.left.data.rowSize);
            memcpy(outDegradedImagePair.left.data.data.arr, outLeft.data, static_cast<size_t>(outDegradedImagePair.left.data.data.nCount));
        }
    }

    // Getting Right image
    {
        // init the structure
        outDegradedImagePair.right.msgVersion = frame_Version;

        outDegradedImagePair.right.intrinsic = inOriginalImagePair.right.intrinsic;

        // Dividing fx, fy, s, cx, cy by Ratio
        outDegradedImagePair.right.intrinsic.cameraMatrix.arr[0].arr[0] = inOriginalImagePair.right.intrinsic.cameraMatrix.arr[0].arr[0]/parameters.xratio;
        outDegradedImagePair.right.intrinsic.cameraMatrix.arr[0].arr[1] = inOriginalImagePair.right.intrinsic.cameraMatrix.arr[0].arr[1]/parameters.xratio;
        outDegradedImagePair.right.intrinsic.cameraMatrix.arr[0].arr[2] = inOriginalImagePair.right.intrinsic.cameraMatrix.arr[0].arr[2]/parameters.xratio;
        outDegradedImagePair.right.intrinsic.cameraMatrix.arr[1].arr[1] = inOriginalImagePair.right.intrinsic.cameraMatrix.arr[1].arr[1]/parameters.yratio;
        outDegradedImagePair.right.intrinsic.cameraMatrix.arr[1].arr[2] = inOriginalImagePair.right.intrinsic.cameraMatrix.arr[1].arr[2]/parameters.yratio;

        outDegradedImagePair.right.extrinsic = inOriginalImagePair.right.extrinsic;
        outDegradedImagePair.right.metadata = inOriginalImagePair.right.metadata;

        // Array3D
        {
            outDegradedImagePair.right.data.msgVersion = array3D_Version;
            outDegradedImagePair.right.data.rows = static_cast<asn1SccT_UInt32>(outRight.rows);
            outDegradedImagePair.right.data.cols = static_cast<asn1SccT_UInt32>(outRight.cols);
            outDegradedImagePair.right.data.channels = static_cast<asn1SccT_UInt32>(outRight.channels());
            outDegradedImagePair.right.data.depth = static_cast<asn1SccArray3D_depth_t>(outRight.depth());
            outDegradedImagePair.right.data.rowSize = static_cast<asn1SccT_UInt32>(outRight.step[0]);
            outDegradedImagePair.right.data.data.nCount = static_cast<int>(outDegradedImagePair.right.data.rows * outDegradedImagePair.right.data.rowSize);
            memcpy(outDegradedImagePair.right.data.data.arr, outRight.data, static_cast<size_t>(outDegradedImagePair.right.data.data.nCount));
        }
    }
}

}
}
}

/** @} */
