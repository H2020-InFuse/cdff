/**
 * @addtogroup DFNs
 * @{
 */

#include "StereoRectification.hpp"
#include <Eigen/Core>
#include <iostream>

namespace CDFF
{
namespace DFN
{
namespace StereoRectification
{

StereoRectification::StereoRectification()
{
    _sensorIdLeft = "";
    _sensorIdRight = "";
    _calibrationFilePath = " ";
    _xratio = 1;
    _yratio = 1;
    _scaling = -1;
    _initialized = false;

    parametersHelper.AddParameter<int>("StereoRectificationParams", "xratio", parameters.xratio, DEFAULT_PARAMETERS.xratio);
    parametersHelper.AddParameter<int>("StereoRectificationParams", "yratio", parameters.yratio, DEFAULT_PARAMETERS.yratio);
    parametersHelper.AddParameter<double>("StereoRectificationParams", "scaling", parameters.scaling, DEFAULT_PARAMETERS.scaling);
    parametersHelper.AddParameter<bool>("StereoRectificationParams", "fisheye", parameters.fisheye, DEFAULT_PARAMETERS.fisheye);
    parametersHelper.AddParameter<std::string>("StereoRectificationParams", "calibrationFilePath", parameters.calibrationFilePath, DEFAULT_PARAMETERS.calibrationFilePath);

    configurationFilePath = "";
}

StereoRectification::~StereoRectification()
{
}

void StereoRectification::configure()
{
    if(configurationFilePath != ""){
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void StereoRectification::process()
{
    cv::Mat inLeft(static_cast<int>(inOriginalStereoPair.left.data.rows), static_cast<int>(inOriginalStereoPair.left.data.cols), CV_MAKETYPE(static_cast<int>(inOriginalStereoPair.left.data.depth), static_cast<int>(inOriginalStereoPair.left.data.channels)), inOriginalStereoPair.left.data.data.arr, inOriginalStereoPair.left.data.rowSize);
    cv::Mat inRight(static_cast<int>(inOriginalStereoPair.right.data.rows), static_cast<int>(inOriginalStereoPair.right.data.cols), CV_MAKETYPE(static_cast<int>(inOriginalStereoPair.right.data.depth), static_cast<int>(inOriginalStereoPair.right.data.channels)), inOriginalStereoPair.right.data.data.arr, inOriginalStereoPair.right.data.rowSize);
    cv::Mat outLeft;
    cv::Mat outRight;

    // Generate correction maps if needed
    if( std::string(reinterpret_cast<char const *>(inOriginalStereoPair.left.intrinsic.sensorId.arr)) != _sensorIdLeft ||
            std::string(reinterpret_cast<char const *>(inOriginalStereoPair.right.intrinsic.sensorId.arr)) != _sensorIdRight ||
            parameters.calibrationFilePath != _calibrationFilePath ||
            parameters.xratio != _xratio ||
            parameters.yratio != _yratio ||
            parameters.scaling != _scaling ||
            parameters.fisheye != _fisheye){

        _sensorIdLeft = std::string(reinterpret_cast<char const *>(inOriginalStereoPair.left.intrinsic.sensorId.arr));
        _sensorIdRight = std::string(reinterpret_cast<char const *>(inOriginalStereoPair.right.intrinsic.sensorId.arr));
        _calibrationFilePath = parameters.calibrationFilePath;
        _xratio = parameters.xratio;
        _yratio = parameters.yratio;
        _scaling = parameters.scaling;
        _fisheye = parameters.fisheye;

        cv::FileStorage fs( _calibrationFilePath + "/" + _sensorIdLeft + std::string("-") + _sensorIdRight + ".yml", cv::FileStorage::READ );
        if( fs.isOpened() ){
            cv::Size imageSize;
            cv::Mat1d cameraMatrixL;
            cv::Mat1d cameraMatrixR;
            cv::Mat1d distCoeffsL;
            cv::Mat1d distCoeffsR;
            cv::Mat1d R;
            cv::Mat1d T;

            fs["image_width"]  >> imageSize.width;
            fs["image_height"] >> imageSize.height;

            fs["camera_matrix_1"] >> cameraMatrixL;
            fs["distortion_coefficients_1"] >> distCoeffsL;

            fs["camera_matrix_2"] >> cameraMatrixR;
            fs["distortion_coefficients_2"] >> distCoeffsR;

            fs["rotation_matrix"] >> R;
            fs["translation_coefficients"] >> T;

            cv::Size newSize;
            newSize.width = imageSize.width / _xratio;
            newSize.height = imageSize.height / _yratio;

            cv::Mat1d RLeft;
            cv::Mat1d RRight;
            cv::Mat1d Q;

            if(_fisheye){
                cv::fisheye::stereoRectify(cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, imageSize, R, T, RLeft, RRight, _PLeft, _PRight, Q, CV_CALIB_ZERO_DISPARITY, newSize, _scaling);

                cv::fisheye::initUndistortRectifyMap(cameraMatrixL, distCoeffsL, RLeft, _PLeft, newSize, CV_32F, _lmapx, _lmapy);
                cv::fisheye::initUndistortRectifyMap(cameraMatrixR, distCoeffsR, RRight, _PRight, newSize, CV_32F, _rmapx, _rmapy);
            }
            else{
                cv::stereoRectify(cameraMatrixL, distCoeffsL, cameraMatrixR, distCoeffsR, imageSize, R, T, RLeft, RRight, _PLeft, _PRight, Q, CV_CALIB_ZERO_DISPARITY, _scaling, newSize);

                cv::initUndistortRectifyMap(cameraMatrixL, distCoeffsL, RLeft, _PLeft, newSize, CV_32F, _lmapx, _lmapy);
                cv::initUndistortRectifyMap(cameraMatrixR, distCoeffsR, RRight, _PRight, newSize, CV_32F, _rmapx, _rmapy);
            }

            _baseline = 1.0 / Q.at<double>(3,2);

            _initialized = true;
        }
        else{
            _initialized = false;
            std::cerr << "Can't open the calibration file: " << _calibrationFilePath + "/" + _sensorIdLeft + std::string("-") + _sensorIdRight + ".yml" << std::endl;
        }
    }

    if( _initialized ){
        cv::remap(inLeft, outLeft, _lmapx, _lmapy, cv::INTER_LINEAR);
        cv::remap(inRight, outRight, _rmapx, _rmapy, cv::INTER_LINEAR);

        // Getting image pair
        outRectifiedStereoPair.msgVersion = frame_Version;
        outRectifiedStereoPair.baseline = _baseline;

        // Left image
        {
            asn1SccFrame & img = outRectifiedStereoPair.left;

            // init the structure
            img.msgVersion = frame_Version;

            img.intrinsic = inOriginalStereoPair.left.intrinsic;

            Eigen::Map<Eigen::Matrix3d>(img.intrinsic.cameraMatrix.arr[0].arr, 3, 3) = Eigen::Map<Eigen::Matrix3d, 0, Eigen::OuterStride<>>(reinterpret_cast<double *>(_PLeft.data), 3, 3, Eigen::OuterStride<>(4));

            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(img.intrinsic.distCoeffs.arr, inOriginalStereoPair.left.intrinsic.distCoeffs.nCount, 1) = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(inOriginalStereoPair.left.intrinsic.distCoeffs.nCount, 1);
            img.intrinsic.distCoeffs.nCount = inOriginalStereoPair.left.intrinsic.distCoeffs.nCount;

            if(_fisheye){
                img.intrinsic.cameraModel = asn1Scccam_FISHEYE;
            }
            else{
                img.intrinsic.cameraModel = asn1Scccam_PINHOLE;
            }

            img.extrinsic = inOriginalStereoPair.left.extrinsic;
            img.metadata = inOriginalStereoPair.left.metadata;

            // Array3D
            {
                img.data.msgVersion = array3D_Version;
                img.data.rows = static_cast<asn1SccT_UInt32>(outLeft.rows);
                img.data.cols = static_cast<asn1SccT_UInt32>(outLeft.cols);
                img.data.channels = static_cast<asn1SccT_UInt32>(outLeft.channels());
                img.data.depth = static_cast<asn1SccArray3D_depth_t>(outLeft.depth());
                img.data.rowSize = static_cast<asn1SccT_UInt32>(outLeft.step[0]);
                img.data.data.nCount = static_cast<int>(img.data.rows * img.data.rowSize);
                memcpy(img.data.data.arr, outLeft.data, static_cast<size_t>(img.data.data.nCount));
            }
        }

        // Right image
        {
            asn1SccFrame & img = outRectifiedStereoPair.right;

            // init the structure
            img.msgVersion = frame_Version;

            img.intrinsic = inOriginalStereoPair.right.intrinsic;

            Eigen::Map<Eigen::Matrix3d>(img.intrinsic.cameraMatrix.arr[0].arr, 3, 3) = Eigen::Map<Eigen::Matrix3d, 0, Eigen::OuterStride<>>(reinterpret_cast<double *>(_PRight.data), 3, 3, Eigen::OuterStride<>(4));

            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(img.intrinsic.distCoeffs.arr, inOriginalStereoPair.right.intrinsic.distCoeffs.nCount, 1) = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(inOriginalStereoPair.right.intrinsic.distCoeffs.nCount, 1);
            img.intrinsic.distCoeffs.nCount = inOriginalStereoPair.right.intrinsic.distCoeffs.nCount;

            if(_fisheye){
                img.intrinsic.cameraModel = asn1Scccam_FISHEYE;
            }
            else{
                img.intrinsic.cameraModel = asn1Scccam_PINHOLE;
            }

            img.extrinsic = inOriginalStereoPair.right.extrinsic;
            img.metadata = inOriginalStereoPair.right.metadata;

            // Array3D
            {
                img.data.msgVersion = array3D_Version;
                img.data.rows = static_cast<asn1SccT_UInt32>(outRight.rows);
                img.data.cols = static_cast<asn1SccT_UInt32>(outRight.cols);
                img.data.channels = static_cast<asn1SccT_UInt32>(outRight.channels());
                img.data.depth = static_cast<asn1SccArray3D_depth_t>(outRight.depth());
                img.data.rowSize = static_cast<asn1SccT_UInt32>(outRight.step[0]);
                img.data.data.nCount = static_cast<int>(img.data.rows * img.data.rowSize);
                memcpy(img.data.data.arr, outRight.data, static_cast<size_t>(img.data.data.nCount));
            }
        }
    }
}

void StereoRectification::ValidateParameters()
{
    ASSERT(parameters.xratio >= 1 && parameters.xratio <= 25, "xratio has to be within [1..25]");
    ASSERT(parameters.yratio >= 1 && parameters.yratio <= 25, "yratio has to be within [1..25]");
    ASSERT((parameters.scaling >= 0 && parameters.scaling <= 1) || parameters.scaling == -1, "scaling has to be within [0..1] or equal to -1");
}

const StereoRectification::StereoRectificationParams StereoRectification::DEFAULT_PARAMETERS = {
    .xratio = 1,
    .yratio = 1,
    .scaling = -1,
    .fisheye = false,
    .calibrationFilePath = ""
};

}
}
}

/** @} */
