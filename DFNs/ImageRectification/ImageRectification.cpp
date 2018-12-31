/**
 * @addtogroup DFNs
 * @{
 */

#include "ImageRectification.hpp"
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
//#include "Asn1Tools.h"
#include <iostream>

namespace CDFF
{
namespace DFN
{
namespace ImageRectification
{

ImageRectification::ImageRectification()
{
    _sensorId = "";
    _xratio = 1;
    _yratio = 1;
    _scaling = -1;
    _centerPrincipalPoint = false;

    parametersHelper.AddParameter<int>("ImageRectificationParams", "xratio", parameters.xratio, DEFAULT_PARAMETERS.xratio);
    parametersHelper.AddParameter<int>("ImageRectificationParams", "yratio", parameters.yratio, DEFAULT_PARAMETERS.yratio);
    parametersHelper.AddParameter<double>("ImageRectificationParams", "scaling", parameters.scaling, DEFAULT_PARAMETERS.scaling);
    parametersHelper.AddParameter<bool>("ImageRectificationParams", "centerPrincipalPoint", parameters.centerPrincipalPoint, DEFAULT_PARAMETERS.centerPrincipalPoint);
    parametersHelper.AddParameter<bool>("ImageRectificationParams", "fisheye", parameters.fisheye, DEFAULT_PARAMETERS.fisheye);

    configurationFilePath = "";
}

ImageRectification::~ImageRectification()
{
}

void ImageRectification::configure()
{
    if(configurationFilePath != ""){
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void ImageRectification::process()
{
    cv::Mat in(static_cast<int>(inOriginalImage.data.rows), static_cast<int>(inOriginalImage.data.cols), CV_MAKETYPE(static_cast<int>(inOriginalImage.data.depth), static_cast<int>(inOriginalImage.data.channels)), inOriginalImage.data.data.arr, inOriginalImage.data.rowSize);
    cv::Mat out;

    // Generate correction maps if needed
    if( std::string(reinterpret_cast<char const *>(inOriginalImage.intrinsic.sensorId.arr)) != _sensorId ||
            parameters.xratio != _xratio ||
            parameters.yratio != _yratio ||
            parameters.scaling != _scaling ||
            parameters.centerPrincipalPoint != _centerPrincipalPoint ||
            parameters.fisheye != _fisheye){
        _sensorId = std::string(reinterpret_cast<char const *>(inOriginalImage.intrinsic.sensorId.arr));
        _xratio = parameters.xratio;
        _yratio = parameters.yratio;
        _scaling = parameters.scaling;
        _centerPrincipalPoint =parameters.centerPrincipalPoint;
        _fisheye = parameters.fisheye;

        cv::Mat1d cameraMatrix(3,3, Eigen::Map<Eigen::Matrix3d>(inOriginalImage.intrinsic.cameraMatrix.arr[0].arr, 3, 3).data());

        cv::Mat1d distCoeffs(inOriginalImage.intrinsic.distCoeffs.nCount, 1, Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(inOriginalImage.intrinsic.distCoeffs.arr, inOriginalImage.intrinsic.distCoeffs.nCount, 1).data());

        if(_fisheye){
            cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, cv::Size(in.cols, in.rows), cv::Matx33d::eye(), _newCameraMatrix, _scaling, cv::Size(in.cols / _xratio, in.rows / _yratio));
            cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Matx33d::eye(), _newCameraMatrix, cv::Size(in.cols / _xratio, in.rows / _yratio), CV_32F, _mapx, _mapy);
        }
        else{
            _newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(in.cols, in.rows), _scaling, cv::Size(in.cols / _xratio, in.rows / _yratio), 0, _centerPrincipalPoint);
            cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), _newCameraMatrix, cv::Size(in.cols / _xratio, in.rows / _yratio), CV_32F, _mapx, _mapy);
        }
    }

    cv::remap(in, out, _mapx, _mapy, cv::INTER_LINEAR);

    // Getting image
    {
        // init the structure
        outRectifiedImage.msgVersion = frame_Version;

        outRectifiedImage.intrinsic = inOriginalImage.intrinsic;

        Eigen::Map<Eigen::Matrix3d>(outRectifiedImage.intrinsic.cameraMatrix.arr[0].arr, 3, 3) = Eigen::Map<Eigen::Matrix3d>(reinterpret_cast<double *>(_newCameraMatrix.data), 3, 3);

        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(outRectifiedImage.intrinsic.distCoeffs.arr, inOriginalImage.intrinsic.distCoeffs.nCount, 1) = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(inOriginalImage.intrinsic.distCoeffs.nCount, 1);
        outRectifiedImage.intrinsic.distCoeffs.nCount = inOriginalImage.intrinsic.distCoeffs.nCount;

        if(_fisheye){
            outRectifiedImage.intrinsic.cameraModel = asn1Scccam_FISHEYE;
        }
        else{
            outRectifiedImage.intrinsic.cameraModel = asn1Scccam_PINHOLE;
        }

        outRectifiedImage.extrinsic = inOriginalImage.extrinsic;
        outRectifiedImage.metadata = inOriginalImage.metadata;

        // Array3D
        {
            outRectifiedImage.data.msgVersion = array3D_Version;
            outRectifiedImage.data.rows = static_cast<asn1SccT_UInt32>(out.rows);
            outRectifiedImage.data.cols = static_cast<asn1SccT_UInt32>(out.cols);
            outRectifiedImage.data.channels = static_cast<asn1SccT_UInt32>(out.channels());
            outRectifiedImage.data.depth = static_cast<asn1SccArray3D_depth_t>(out.depth());
            outRectifiedImage.data.rowSize = static_cast<asn1SccT_UInt32>(out.step[0]);
            outRectifiedImage.data.data.nCount = static_cast<int>(outRectifiedImage.data.rows * outRectifiedImage.data.rowSize);
            memcpy(outRectifiedImage.data.data.arr, out.data, static_cast<size_t>(outRectifiedImage.data.data.nCount));
        }
    }

}

void ImageRectification::ValidateParameters()
{
    ASSERT(parameters.xratio >= 1 && parameters.xratio <= 25, "xratio has to be within [1..25]");
    ASSERT(parameters.yratio >= 1 && parameters.yratio <= 25, "yratio has to be within [1..25]");
    ASSERT((parameters.scaling >= 0 && parameters.scaling <= 1) || parameters.scaling == -1, "scaling has to be within [0..1] or equal to -1");
}

const ImageRectification::ImageRectificationParams ImageRectification::DEFAULT_PARAMETERS = {
    .xratio = 1,
    .yratio = 1,
    .scaling = -1,
    .centerPrincipalPoint = false
};

}
}
}

/** @} */
