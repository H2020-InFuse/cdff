/**
 * @addtogroup DFNs
 * @{
 */

#include "ImageDegradation.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace CDFF
{
namespace DFN
{
namespace ImageDegradation
{

ImageDegradation::ImageDegradation()
{
    parametersHelper.AddParameter<int>("ImageDegradationParams", "xratio", parameters.xratio, DEFAULT_PARAMETERS.xratio);
    parametersHelper.AddParameter<int>("ImageDegradationParams", "yratio", parameters.yratio, DEFAULT_PARAMETERS.yratio);
    parametersHelper.AddParameter<int>("ImageDegradationParams", "method", parameters.method, DEFAULT_PARAMETERS.method);

    configurationFilePath = "";
}

ImageDegradation::~ImageDegradation()
{
}

void ImageDegradation::configure()
{
    if(configurationFilePath != ""){
      parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void ImageDegradation::process()
{
    cv::Mat in(static_cast<int>(inOriginalImage.data.rows), static_cast<int>(inOriginalImage.data.cols), CV_MAKETYPE(static_cast<int>(inOriginalImage.data.depth), static_cast<int>(inOriginalImage.data.channels)), inOriginalImage.data.data.arr, inOriginalImage.data.rowSize);
    cv::Mat out;

    cv::resize(in, out, cv::Size(in.cols / parameters.xratio, in.rows / parameters.yratio), 0, 0, static_cast<cv::InterpolationFlags>(parameters.method));

    // Getting image
    {
        // init the structure
        outDegradedImage.msgVersion = frame_Version;

        outDegradedImage.intrinsic = inOriginalImage.intrinsic;

        // Dividing fx, fy, s, cx, cy by Ratio
        outDegradedImage.intrinsic.cameraMatrix.arr[0].arr[0] = inOriginalImage.intrinsic.cameraMatrix.arr[0].arr[0]/parameters.xratio;
        outDegradedImage.intrinsic.cameraMatrix.arr[0].arr[1] = inOriginalImage.intrinsic.cameraMatrix.arr[0].arr[1]/parameters.xratio;
        outDegradedImage.intrinsic.cameraMatrix.arr[0].arr[2] = inOriginalImage.intrinsic.cameraMatrix.arr[0].arr[2]/parameters.xratio;
        outDegradedImage.intrinsic.cameraMatrix.arr[1].arr[1] = inOriginalImage.intrinsic.cameraMatrix.arr[1].arr[1]/parameters.yratio;
        outDegradedImage.intrinsic.cameraMatrix.arr[1].arr[2] = inOriginalImage.intrinsic.cameraMatrix.arr[1].arr[2]/parameters.yratio;

        outDegradedImage.extrinsic = inOriginalImage.extrinsic;
        outDegradedImage.metadata = inOriginalImage.metadata;

        // Array3D
        {
            outDegradedImage.data.msgVersion = array3D_Version;
            outDegradedImage.data.rows = static_cast<asn1SccT_UInt32>(out.rows);
            outDegradedImage.data.cols = static_cast<asn1SccT_UInt32>(out.cols);
            outDegradedImage.data.channels = static_cast<asn1SccT_UInt32>(out.channels());
            outDegradedImage.data.depth = static_cast<asn1SccArray3D_depth_t>(out.depth());
            outDegradedImage.data.rowSize = static_cast<asn1SccT_UInt32>(out.step[0]);
            outDegradedImage.data.data.nCount = static_cast<int>(outDegradedImage.data.rows * outDegradedImage.data.rowSize);
            memcpy(outDegradedImage.data.data.arr, out.data, static_cast<size_t>(outDegradedImage.data.data.nCount));
        }
    }
}

void ImageDegradation::ValidateParameters()
{
    ASSERT(parameters.xratio >= 1 && parameters.xratio <= 25, "xratio has to be within [1..25]");
    ASSERT(parameters.yratio >= 1 && parameters.yratio <= 25, "yratio has to be within [1..25]");
    ASSERT(parameters.method >= 0 || parameters.method <= 5 || parameters.method == 7, "method has to be 0, 1, 2, 3, 4, 5, or 7");
}

const ImageDegradation::ImageDegradationParams ImageDegradation::DEFAULT_PARAMETERS = {
    .xratio = 2,
    .yratio = 2,
    .method = 3
};

}
}
}

/** @} */
