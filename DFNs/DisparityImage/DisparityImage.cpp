/**
 * @addtogroup DFNs
 * @{
 */

#include "DisparityImage.hpp"

namespace CDFF
{
namespace DFN
{
namespace DisparityImage
{

DisparityImage::DisparityImage()
{
    parametersHelper.AddParameter<int>("stereoMatcherParams", "algorithm", parameters.stereoMatcher.algorithm, DEFAULT_PARAMETERS.stereoMatcher.algorithm);
    parametersHelper.AddParameter<int>("stereoMatcherParams", "minDisparity", parameters.stereoMatcher.minDisparity, DEFAULT_PARAMETERS.stereoMatcher.minDisparity);
    parametersHelper.AddParameter<int>("stereoMatcherParams", "numDisparities", parameters.stereoMatcher.numDisparities, DEFAULT_PARAMETERS.stereoMatcher.numDisparities);
    parametersHelper.AddParameter<int>("stereoMatcherParams", "blockSize", parameters.stereoMatcher.blockSize, DEFAULT_PARAMETERS.stereoMatcher.blockSize);
    parametersHelper.AddParameter<int>("stereoMatcherParams", "speckleWindowSize", parameters.stereoMatcher.speckleWindowSize, DEFAULT_PARAMETERS.stereoMatcher.speckleWindowSize);
    parametersHelper.AddParameter<int>("stereoMatcherParams", "speckleRange", parameters.stereoMatcher.speckleRange, DEFAULT_PARAMETERS.stereoMatcher.speckleRange);
    parametersHelper.AddParameter<int>("stereoMatcherParams", "disp12MaxDiff", parameters.stereoMatcher.disp12MaxDiff, DEFAULT_PARAMETERS.stereoMatcher.disp12MaxDiff);
    parametersHelper.AddParameter<int>("stereoMatcherParams", "preFilterCap", parameters.stereoMatcher.preFilterCap, DEFAULT_PARAMETERS.stereoMatcher.preFilterCap);
    parametersHelper.AddParameter<int>("stereoMatcherParams", "uniquenessRatio", parameters.stereoMatcher.uniquenessRatio, DEFAULT_PARAMETERS.stereoMatcher.uniquenessRatio);

    parametersHelper.AddParameter<int>("stereoBMParams", "preFilterType", parameters.stereoMatcher.bmParams.preFilterType, DEFAULT_PARAMETERS.stereoMatcher.bmParams.preFilterType);
    parametersHelper.AddParameter<int>("stereoBMParams", "preFilterSize", parameters.stereoMatcher.bmParams.preFilterSize, DEFAULT_PARAMETERS.stereoMatcher.bmParams.preFilterSize);
    parametersHelper.AddParameter<int>("stereoBMParams", "textureThreshold", parameters.stereoMatcher.bmParams.textureThreshold, DEFAULT_PARAMETERS.stereoMatcher.bmParams.textureThreshold);

    parametersHelper.AddParameter<int>("stereoSGBMParams", "P1", parameters.stereoMatcher.sgbmParams.P1, DEFAULT_PARAMETERS.stereoMatcher.sgbmParams.P1);
    parametersHelper.AddParameter<int>("stereoSGBMParams", "P2", parameters.stereoMatcher.sgbmParams.P2, DEFAULT_PARAMETERS.stereoMatcher.sgbmParams.P2);
    parametersHelper.AddParameter<int>("stereoSGBMParams", "mode", parameters.stereoMatcher.sgbmParams.mode, DEFAULT_PARAMETERS.stereoMatcher.sgbmParams.mode);

#if WITH_XIMGPROC
    parametersHelper.AddParameter<bool>("filterParams", "useFilter", parameters.filter.useFilter, DEFAULT_PARAMETERS.filter.useFilter);
    parametersHelper.AddParameter<bool>("filterParams", "useConfidence", parameters.filter.useConfidence, DEFAULT_PARAMETERS.filter.useConfidence);
    parametersHelper.AddParameter<int>("filterParams", "depthDiscontinuityRadius", parameters.filter.depthDiscontinuityRadius, DEFAULT_PARAMETERS.filter.depthDiscontinuityRadius);
    parametersHelper.AddParameter<double>("filterParams", "lambda", parameters.filter.lambda, DEFAULT_PARAMETERS.filter.lambda);
    parametersHelper.AddParameter<int>("filterParams", "lrcThresh", parameters.filter.lrcThresh, DEFAULT_PARAMETERS.filter.lrcThresh);
    parametersHelper.AddParameter<double>("filterParams", "sigmaColor", parameters.filter.sigmaColor, DEFAULT_PARAMETERS.filter.sigmaColor);

    _algorithm = -1;
    _useConfidence = false;
#endif

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
    cv::Mat imgLeft(static_cast<int>(inFramePair.left.data.rows), static_cast<int>(inFramePair.left.data.cols), CV_MAKETYPE(static_cast<int>(inFramePair.left.data.depth), static_cast<int>(inFramePair.left.data.channels)), inFramePair.left.data.data.arr, inFramePair.left.data.rowSize);
    cv::Mat imgRight(static_cast<int>(inFramePair.right.data.rows), static_cast<int>(inFramePair.right.data.cols), CV_MAKETYPE(static_cast<int>(inFramePair.right.data.depth), static_cast<int>(inFramePair.right.data.channels)), inFramePair.right.data.data.arr, inFramePair.right.data.rowSize);
    cv::Mat disparity;

    // Using Algorithm StereoBM
    if(parameters.stereoMatcher.algorithm == 0){
        if(_bm.empty()){
            _bm = cv::StereoBM::create(parameters.stereoMatcher.numDisparities, parameters.stereoMatcher.blockSize);
        }

        _bm->setBlockSize(parameters.stereoMatcher.blockSize);
        _bm->setDisp12MaxDiff(parameters.stereoMatcher.disp12MaxDiff);
        _bm->setMinDisparity(parameters.stereoMatcher.minDisparity);
        _bm->setNumDisparities(parameters.stereoMatcher.numDisparities);
        _bm->setPreFilterCap(parameters.stereoMatcher.preFilterCap);
        _bm->setPreFilterSize(parameters.stereoMatcher.bmParams.preFilterSize);
        _bm->setPreFilterType(parameters.stereoMatcher.bmParams.preFilterType);
        _bm->setSpeckleRange(parameters.stereoMatcher.speckleRange);
        _bm->setSpeckleWindowSize(parameters.stereoMatcher.speckleWindowSize);
        _bm->setTextureThreshold(parameters.stereoMatcher.bmParams.textureThreshold);
        _bm->setUniquenessRatio(parameters.stereoMatcher.uniquenessRatio);

        _bm->compute(imgLeft, imgRight, disparity);

    }
    // Using Algorithm StereoSGBM
    else if(parameters.stereoMatcher.algorithm == 1){
        if(_sgbm.empty()){
            _sgbm = cv::StereoSGBM::create(parameters.stereoMatcher.minDisparity, parameters.stereoMatcher.numDisparities, parameters.stereoMatcher.blockSize, parameters.stereoMatcher.sgbmParams.P1, parameters.stereoMatcher.sgbmParams.P2, parameters.stereoMatcher.disp12MaxDiff, parameters.stereoMatcher.preFilterCap, parameters.stereoMatcher.uniquenessRatio, parameters.stereoMatcher.speckleWindowSize, parameters.stereoMatcher.speckleRange, parameters.stereoMatcher.sgbmParams.mode);
        }

        _sgbm->setBlockSize(parameters.stereoMatcher.blockSize);
        _sgbm->setDisp12MaxDiff(parameters.stereoMatcher.disp12MaxDiff);
        _sgbm->setMinDisparity(parameters.stereoMatcher.minDisparity);
        _sgbm->setMode(parameters.stereoMatcher.sgbmParams.mode);
        _sgbm->setNumDisparities(parameters.stereoMatcher.numDisparities);
        _sgbm->setP1(parameters.stereoMatcher.sgbmParams.P1);
        _sgbm->setP2(parameters.stereoMatcher.sgbmParams.P2);
        _sgbm->setPreFilterCap(parameters.stereoMatcher.preFilterCap);
        _sgbm->setSpeckleRange(parameters.stereoMatcher.speckleRange);
        _sgbm->setSpeckleWindowSize(parameters.stereoMatcher.speckleWindowSize);
        _sgbm->setUniquenessRatio(parameters.stereoMatcher.uniquenessRatio);

        _sgbm->compute(imgLeft, imgRight, disparity);
    }

#if WITH_XIMGPROC
    bool resetFilter = false;
    bool resetMatcher = false;
    if(parameters.stereoMatcher.algorithm != _algorithm){
        _algorithm = parameters.stereoMatcher.algorithm;
        resetFilter = true;
        resetMatcher = true;
    }
    else if(parameters.filter.useConfidence != _useConfidence){
        _useConfidence = parameters.filter.useConfidence;
        resetFilter = true;
    }

    if(parameters.filter.useFilter){
        cv::Mat disparityFiltered;

        if(parameters.filter.useConfidence){
            cv::Mat disparityRight;
            if(_rightMatcher.empty() || resetMatcher){
                switch(_algorithm){
                case 0:
                    _rightMatcher = cv::ximgproc::createRightMatcher(_bm);
                    break;
                case 1:
                    _rightMatcher = cv::ximgproc::createRightMatcher(_sgbm);
                    break;
                }
            }

            _rightMatcher->compute(imgRight, imgLeft, disparityRight);

            if(_filter.empty() || resetFilter){
                switch(_algorithm){
                case 0:
                    _filter = cv::ximgproc::createDisparityWLSFilter(_bm);
                    break;
                case 1:
                    _filter = cv::ximgproc::createDisparityWLSFilter(_sgbm);
                    break;
                }
            }

            _filter->setDepthDiscontinuityRadius(parameters.filter.depthDiscontinuityRadius);
            _filter->setLambda(parameters.filter.lambda);
            _filter->setLRCthresh(parameters.filter.lrcThresh);
            _filter->setSigmaColor(parameters.filter.sigmaColor);

            _filter->filter(disparity, imgLeft, disparityFiltered, disparityRight);
        }
        else{
            if(_filter.empty() || resetFilter){
                _filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
            }

            _filter->setDepthDiscontinuityRadius(parameters.filter.depthDiscontinuityRadius);
            _filter->setLambda(parameters.filter.lambda);
            _filter->setSigmaColor(parameters.filter.sigmaColor);

            _filter->filter(disparity, imgLeft, disparityFiltered);
        }

        disparity = disparityFiltered;
    }
#endif

    // Convert Mat to ASN.1
    outRawDisparity.metadata.msgVersion = frame_Version;
    outRawDisparity.metadata = inFramePair.left.metadata;
    outRawDisparity.intrinsic = inFramePair.left.intrinsic;
    outRawDisparity.extrinsic = inFramePair.left.extrinsic;

    outRawDisparity.metadata.mode = asn1Sccmode_UNDEF;
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
    outRawDisparity.data.channels = static_cast<asn1SccT_UInt32>(disparity.channels());
    outRawDisparity.data.rows = static_cast<asn1SccT_UInt32>(inFramePair.left.data.rows);
    outRawDisparity.data.cols = static_cast<asn1SccT_UInt32>(inFramePair.left.data.cols);
    outRawDisparity.data.depth = static_cast<asn1SccArray3D_depth_t>(disparity.depth());
    outRawDisparity.data.rowSize = disparity.step[0];
    outRawDisparity.data.data.nCount =  static_cast<int>(outRawDisparity.data.rows * outRawDisparity.data.rowSize);
    memcpy(outRawDisparity.data.data.arr, disparity.data, static_cast<size_t>(outRawDisparity.data.data.nCount));
}

void DisparityImage::ValidateParameters()
{
    ASSERT(parameters.stereoMatcher.algorithm >= 0 && parameters.stereoMatcher.algorithm <= 1, "stereoMatcher.algorithm must be 0 or 1");
    ASSERT(parameters.stereoMatcher.minDisparity >= 0, "stereoMatcher.minDisparity has to be positive");
    ASSERT(parameters.stereoMatcher.numDisparities > 0 && parameters.stereoMatcher.numDisparities %16 == 0, "stereoMatcher.numDisparities has to be greater than 0 and divisible by 16");
    if(parameters.stereoMatcher.algorithm == 0){
        ASSERT(parameters.stereoMatcher.blockSize %2 != 0 && parameters.stereoMatcher.blockSize >= 5 && parameters.stereoMatcher.blockSize <= 255, "stereoMatcher.blockSize must be an odd number in range [5..255]");
    }
    else if(parameters.stereoMatcher.algorithm == 1){
        ASSERT(parameters.stereoMatcher.blockSize %2 != 0 && parameters.stereoMatcher.blockSize > 0, "stereoMatcher.blockSize must be an odd number >= 1");
    }
    ASSERT(parameters.stereoMatcher.speckleWindowSize >= 0, "stereoMatcher.speckleWindowSize must be positive");
    ASSERT(parameters.stereoMatcher.preFilterCap >= 1 && parameters.stereoMatcher.preFilterCap <= 63 , "stereoMatcher.preFilterCap must be in the [1..63] range");
    ASSERT(parameters.stereoMatcher.uniquenessRatio >= 0, "stereoMatcher.uniquenessRatio must be positive");
    ASSERT(parameters.stereoMatcher.bmParams.preFilterSize %2 != 0 && parameters.stereoMatcher.bmParams.preFilterSize >= 5 && parameters.stereoMatcher.bmParams.preFilterSize <= 255, "stereoMatcher.bmParams.preFilterSize must be an odd number in the [5..255] range");
    ASSERT(parameters.stereoMatcher.bmParams.preFilterType == cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE || parameters.stereoMatcher.bmParams.preFilterType == cv::StereoBM::PREFILTER_XSOBEL, "stereoMatcher.bmParams.preFilterType must be 0 or 1");
    ASSERT(parameters.stereoMatcher.bmParams.textureThreshold >= 0, "stereoMatcher.bmParams.textureThreshold must be positive");
    ASSERT(parameters.stereoMatcher.sgbmParams.P1 >= 0, "stereoMatcher.sgbmParams.P1 must be positive");
    if(parameters.stereoMatcher.sgbmParams.P1 == 0){
        ASSERT(parameters.stereoMatcher.sgbmParams.P2 >= 0, "stereoMatcher.sgbmParams.P2 must be positive");
    }
    else{
        ASSERT(parameters.stereoMatcher.sgbmParams.P2 >= 0 && parameters.stereoMatcher.sgbmParams.P2 > parameters.stereoMatcher.sgbmParams.P1, "stereoMatcher.sgbmParams.P2 must be positive & stereoMatcher.sgbmParams.P2 > stereoMatcher.sgbmParams.P1");
    }
    ASSERT(parameters.stereoMatcher.sgbmParams.mode >= cv::StereoSGBM::MODE_SGBM && parameters.stereoMatcher.sgbmParams.mode <= cv::StereoSGBM::MODE_HH4, "stereoMatcher.sgbmParams.mode must be in the [0..3] range");

#if WITH_XIMGPROC
    ASSERT(parameters.filter.depthDiscontinuityRadius >= 0, "filter.depthDiscontinuityRadius must be positive");
    ASSERT(parameters.filter.lambda >= 0, "filter.lambda must be positive");
    ASSERT(parameters.filter.lrcThresh >= 0, "filter.lrcThresh must be positive");
    ASSERT(parameters.filter.sigmaColor >= 0, "filter.sigmaColor must be positive");
#endif
}

const DisparityImage::DisparityImageParams DisparityImage::DEFAULT_PARAMETERS = {
    {
        .algorithm = 1,
        .minDisparity = 0,
        .numDisparities = 192,
        .blockSize = 5,
        .speckleWindowSize = 0,
        .speckleRange = 0,
        .disp12MaxDiff = -1,
        .preFilterCap = 31,
        .uniquenessRatio = 15,
        {
            .preFilterType = cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE,
            .preFilterSize = 9,
            .textureThreshold = 10
        },
        {
            .P1 = 0,
            .P2 = 0,
            .mode = cv::StereoSGBM::MODE_SGBM
        }
    }
    #if WITH_XIMGPROC
    ,
    {
        .useFilter = false,
        .useConfidence = false,
        .depthDiscontinuityRadius = 0,
        .lambda = 8000.0,
        .lrcThresh = 24,
        .sigmaColor = 1.5
    }
    #endif
};

}
}
}

/** @} */
