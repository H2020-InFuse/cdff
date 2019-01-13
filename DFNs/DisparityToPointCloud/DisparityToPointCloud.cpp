/**
 * @addtogroup DFNs
 * @{
 */

#include "DisparityToPointCloud.hpp"
#include <opencv2/core.hpp>
#include <Errors/Assert.hpp>

namespace CDFF
{
namespace DFN
{
namespace DisparityToPointCloud
{

DisparityToPointCloud::DisparityToPointCloud()
{
}

DisparityToPointCloud::~DisparityToPointCloud()
{
}

void DisparityToPointCloud::configure()
{
}

void DisparityToPointCloud::process()
{
    switch(inDispImage.data.depth)
    {
        case asn1Sccdepth_8U:{
            disp2ptcloud<unsigned char>(inDispImage, outPointCloud);
            break;
        }
        case asn1Sccdepth_8S:{
            disp2ptcloud<char>(inDispImage, outPointCloud);
            break;
        }
        case asn1Sccdepth_16U:{
            disp2ptcloud<unsigned short int>(inDispImage, outPointCloud);
            break;
        }
        case asn1Sccdepth_16S:{
            disp2ptcloud<short int>(inDispImage, outPointCloud);
            break;
        }
        case asn1Sccdepth_32S:{
            disp2ptcloud<int>(inDispImage, outPointCloud);
            break;
        }
        case asn1Sccdepth_32F:{
            disp2ptcloud<float>(inDispImage, outPointCloud);
            break;
        }
        case asn1Sccdepth_64F:{
            disp2ptcloud<double>(inDispImage, outPointCloud);
            break;
        }
    }
}

template<typename T>
bool DisparityToPointCloud::disp2ptcloud(asn1SccFrame &disp, asn1SccPointcloud &ptCloud)
{
    if( disp.metadata.pixelModel != asn1Sccpix_DISP ){
       	PRINT_ERROR("DisparityToPointCloud: Bad input data");
        return false;
    }

    if( (disp.data.rows * disp.data.cols) > maxPointcloudSize ){
        PRINT_ERROR("DisparityToPointCloud: Too many pixels to reproject, image needs to be degraded first");
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
    ptCloud.data.intensity.nCount = 0;

    cv::Mat tmp(static_cast<int>(disp.data.rows), static_cast<int>(disp.data.cols), CV_MAKETYPE(static_cast<int>(disp.data.depth), static_cast<int>(disp.data.channels)), disp.data.data.arr, disp.data.rowSize);

    ptCloud.data.points.nCount = 0;
    for (int i = 0; i < tmp.rows; i++)
    {
        for (int j = 0; j < tmp.cols; j++)
        {
            ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[2] = (disp.metadata.pixelCoeffs.arr[2] * disp.intrinsic.cameraMatrix.arr[0].arr[0]) / ((tmp.at<T>(i, j) * disp.metadata.pixelCoeffs.arr[0]) + disp.metadata.pixelCoeffs.arr[1]);
            ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[0] = ((j - disp.intrinsic.cameraMatrix.arr[0].arr[2]) / disp.intrinsic.cameraMatrix.arr[0].arr[0]) * ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[2];
            ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[1] = ((i - disp.intrinsic.cameraMatrix.arr[1].arr[2]) / disp.intrinsic.cameraMatrix.arr[1].arr[1]) * ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[2];
            
            if (std::isinf(ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[0]) || std::isinf(ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[1]) || std::isinf(ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[2]))
            {
                ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[2] = 0;
                ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[0] = 0;
                ptCloud.data.points.arr[ptCloud.data.points.nCount].arr[1] = 0;
            }
            ptCloud.data.points.nCount++;
        }
    }

    return true;
    }
}

}
}

/** @} */
