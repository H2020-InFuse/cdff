/**
 * @addtogroup DFNs
 * @{
 */

#include "DisparityToPointCloudWithIntensity.hpp"
#include <opencv2/core.hpp>
#include <Errors/Assert.hpp>

namespace CDFF
{
namespace DFN
{
namespace DisparityToPointCloudWithIntensity
{

DisparityToPointCloudWithIntensity::DisparityToPointCloudWithIntensity()
{
}

DisparityToPointCloudWithIntensity::~DisparityToPointCloudWithIntensity()
{
}

void DisparityToPointCloudWithIntensity::configure()
{
}

void DisparityToPointCloudWithIntensity::process()
{
    switch (inDispImage.data.depth)
    {
        case asn1Sccdepth_8U:
        {
            disp2ptcloudwithintensity<unsigned char>(inDispImage, inIntensityImage, outPointCloud);
            break;
        }
        case asn1Sccdepth_8S:
        {
            disp2ptcloudwithintensity<char>(inDispImage, inIntensityImage, outPointCloud);
            break;
        }
        case asn1Sccdepth_16U:
        {
            disp2ptcloudwithintensity<unsigned short int>(inDispImage, inIntensityImage, outPointCloud);
            break;
        }
        case asn1Sccdepth_16S:
        {
            disp2ptcloudwithintensity<short int>(inDispImage, inIntensityImage, outPointCloud);
            break;
        }
        case asn1Sccdepth_32S:
        {
            disp2ptcloudwithintensity<int>(inDispImage, inIntensityImage, outPointCloud);
            break;
        }
        case asn1Sccdepth_32F:
        {
            disp2ptcloudwithintensity<float>(inDispImage, inIntensityImage, outPointCloud);
            break;
        }
        case asn1Sccdepth_64F:
        {
            disp2ptcloudwithintensity<double>(inDispImage, inIntensityImage, outPointCloud);
            break;
        }
    }
}

template <typename T>
bool DisparityToPointCloudWithIntensity::disp2ptcloudwithintensity(asn1SccFrame &disp, asn1SccFrame &img, asn1SccPointcloud &ptCloud)
{
    if (disp.metadata.pixelModel != asn1Sccpix_DISP)
    {
        PRINT_ERROR("DisparityToPointcloud: Bad input data");
        return false;
    }
    else if (img.data.rows != disp.data.rows || img.data.cols != disp.data.cols)
    {
        PRINT_ERROR("DisparityToPointcloud: Disparity and Intensity images must have the same size");

        return false;
    }
    else if (img.data.depth != asn1Sccdepth_8U)
    {
        PRINT_ERROR("DisparityToPointcloud: Intensity image's depth must be 8U ");

        return false;
    }

    if ((disp.data.rows * disp.data.cols) > maxPointcloudSize)
    {
        PRINT_ERROR("DisparityToPointcloud: Too many pixels to reproject, image needs to be degraded first");

        return false;
    }

    ptCloud.metadata.msgVersion = pointCloud_Version;
    ptCloud.metadata.sensorId = disp.intrinsic.sensorId;
    ptCloud.metadata.frameId = disp.extrinsic.pose_robotFrame_sensorFrame.metadata.childFrameId;
    ptCloud.metadata.timeStamp = disp.metadata.timeStamp;
    ptCloud.metadata.isRegistered = false;
    ptCloud.metadata.isOrdered = true;
    ptCloud.metadata.height = disp.data.rows;
    ptCloud.metadata.width = disp.data.cols;
    ptCloud.metadata.hasFixedTransform = disp.extrinsic.hasFixedTransform;
    ptCloud.metadata.pose_robotFrame_sensorFrame = disp.extrinsic.pose_robotFrame_sensorFrame;
    ptCloud.metadata.pose_fixedFrame_robotFrame = disp.extrinsic.pose_fixedFrame_robotFrame;
    ptCloud.data.colors.nCount = 0;

    cv::Mat tmpDisp(static_cast<int>(disp.data.rows), static_cast<int>(disp.data.cols), CV_MAKETYPE(static_cast<int>(disp.data.depth), static_cast<int>(disp.data.channels)), disp.data.data.arr, disp.data.rowSize);
    cv::Mat tmpIntensity(static_cast<int>(img.data.rows), static_cast<int>(img.data.cols), CV_MAKETYPE(static_cast<int>(img.data.depth), static_cast<int>(img.data.channels)), img.data.data.arr, img.data.rowSize);
    
    ptCloud.data.points.nCount = 0;

    for (int i = 0; i < tmpDisp.rows; i++)
    {
        for (int j = 0; j < tmpDisp.cols; j++)
        {
            ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[2] = (disp.metadata.pixelCoeffs.arr[2] * disp.intrinsic.cameraMatrix.arr[0].arr[0]) / ((tmpDisp.at<T>(i, j) * disp.metadata.pixelCoeffs.arr[0]) + disp.metadata.pixelCoeffs.arr[1]);
            ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[0] = ((j - disp.intrinsic.cameraMatrix.arr[0].arr[2]) / disp.intrinsic.cameraMatrix.arr[0].arr[0]) * ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[2];
            ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[1] = ((i - disp.intrinsic.cameraMatrix.arr[1].arr[2]) / disp.intrinsic.cameraMatrix.arr[1].arr[1]) * ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[2];
            ptCloud.data.intensity.arr[ptCloud.data.points.nCount] = static_cast<asn1SccT_Int32>(tmpIntensity.at<unsigned char>(i, j));

            if (std::isinf(ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[0]) || std::isinf(ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[1]) || std::isinf(ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[2]))
            {
                ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[2] = 0;
                ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[0] = 0;
                ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[1] = 0;
            }
            ptCloud.data.points.nCount++;
            ptCloud.data.intensity.nCount = ptCloud.data.points.nCount;
        }
    }

    return true;
}

}
}
}

/** @} */
