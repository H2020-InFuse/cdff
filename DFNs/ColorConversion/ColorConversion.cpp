/**
 * @addtogroup DFNs
 * @{
 */

#include "ColorConversion.hpp"
#include <iostream>

namespace CDFF
{
namespace DFN
{
namespace ColorConversion
{

ColorConversion::ColorConversion()
: parameters(DEFAULT_PARAMETERS)
{
    parametersHelper.AddParameter<int>("ColorConversionParams", "targetMode", parameters.targetMode, DEFAULT_PARAMETERS.targetMode);

    configurationFilePath = "";
}

ColorConversion::~ColorConversion()
{
}

void ColorConversion::configure()
{
    if(configurationFilePath != ""){
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void ColorConversion::process()
{
    _out = cv::Mat();

    if( inOriginalImage.metadata.mode != static_cast<asn1SccFrame_mode_t>(parameters.targetMode)){
        _in = cv::Mat(static_cast<int>(inOriginalImage.data.rows), static_cast<int>(inOriginalImage.data.cols), CV_MAKETYPE(static_cast<int>(inOriginalImage.data.depth), static_cast<int>(inOriginalImage.data.channels)), inOriginalImage.data.data.arr, inOriginalImage.data.rowSize);

        switch(static_cast<asn1SccFrame_mode_t>(parameters.targetMode)){
        case asn1Sccmode_GRAY:
            convertToGray();
            break;
        case asn1Sccmode_RGB:
            convertToRGB();
            break;
        case asn1Sccmode_RGBA:
            convertToRGBA();
            break;
        case asn1Sccmode_BGR:
            convertToBGR();
            break;
        case asn1Sccmode_BGRA:
            convertToBGRA();
            break;
        case asn1Sccmode_HSV:
            convertToHSV();
            break;
        case asn1Sccmode_HLS:
            convertToHLS();
            break;
        case asn1Sccmode_YUV:
            convertToYUV();
            break;
        case asn1Sccmode_Lab:
            convertToLab();
            break;
        case asn1Sccmode_Luv:
            convertToLuv();
            break;
        case asn1Sccmode_XYZ:
            convertToXYZ();
            break;
        case asn1Sccmode_YCrCb:
            convertToYCrCb();
            break;
        default:
            std::cerr << "Bad target mode. Image can't be converted." << std::endl;
            break;
        }
    }

    if( _out.empty() ){
        outConvertedImage = inOriginalImage;
    }
    else{
        // Getting image
        // init the structure
        outConvertedImage.msgVersion = frame_Version;

        outConvertedImage.intrinsic = inOriginalImage.intrinsic;
        outConvertedImage.extrinsic = inOriginalImage.extrinsic;
        outConvertedImage.metadata = inOriginalImage.metadata;
        outConvertedImage.metadata.mode = static_cast<asn1SccFrame_mode_t>(parameters.targetMode);

        // Array3D
        {
            outConvertedImage.data.msgVersion = array3D_Version;
            outConvertedImage.data.rows = static_cast<asn1SccT_UInt32>(_out.rows);
            outConvertedImage.data.cols = static_cast<asn1SccT_UInt32>(_out.cols);
            outConvertedImage.data.channels = static_cast<asn1SccT_UInt32>(_out.channels());
            outConvertedImage.data.depth = static_cast<asn1SccArray3D_depth_t>(_out.depth());
            outConvertedImage.data.rowSize = static_cast<asn1SccT_UInt32>(_out.step[0]);
            outConvertedImage.data.data.nCount = static_cast<int>(outConvertedImage.data.rows * outConvertedImage.data.rowSize);
            memcpy(outConvertedImage.data.data.arr, _out.data, static_cast<size_t>(outConvertedImage.data.data.nCount));
        }
    }
}

void ColorConversion::convertToGray(){
    switch(inOriginalImage.metadata.mode){
    case asn1Sccmode_RGB:
        cv::cvtColor(_in, _out, cv::COLOR_RGB2GRAY);
        break;
    case asn1Sccmode_RGBA:
        cv::cvtColor(_in, _out, cv::COLOR_RGBA2GRAY);
        break;
    case asn1Sccmode_BGR:
        cv::cvtColor(_in, _out, cv::COLOR_BGR2GRAY);
        break;
    case asn1Sccmode_BGRA:
        cv::cvtColor(_in, _out, cv::COLOR_BGRA2GRAY);
        break;
    case asn1Sccmode_HSV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HSV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2GRAY);
        break;
    }
    case asn1Sccmode_HLS:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HLS2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2GRAY);
        break;
    }
    case asn1Sccmode_YUV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YUV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2GRAY);
        break;
    }
    case asn1Sccmode_Lab:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Lab2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2GRAY);
        break;
    }
    case asn1Sccmode_Luv:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Luv2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2GRAY);
        break;
    }
    case asn1Sccmode_XYZ:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_XYZ2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2GRAY);
        break;
    }
    case asn1Sccmode_YCrCb:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YCrCb2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2GRAY);
        break;
    }
    case asn1Sccmode_Bayer_RGGB:
        cv::cvtColor(_in, _out, cv::COLOR_BayerRG2GRAY);
        break;
    case asn1Sccmode_Bayer_GRBG:
        cv::cvtColor(_in, _out, cv::COLOR_BayerGR2GRAY);
        break;
    case asn1Sccmode_Bayer_BGGR:
        cv::cvtColor(_in, _out, cv::COLOR_BayerBG2GRAY);
        break;
    case asn1Sccmode_Bayer_GBRG:
        cv::cvtColor(_in, _out, cv::COLOR_BayerGB2GRAY);
        break;
    default:
        std::cerr << "Bad input asn1Sccmode. Image can't be converted." << std::endl;
        break;
    }
}

void ColorConversion::convertToRGB(){
    switch(inOriginalImage.metadata.mode){
    case asn1Sccmode_GRAY:
        cv::cvtColor(_in, _out, cv::COLOR_GRAY2RGB);
        break;
    case asn1Sccmode_RGBA:
        cv::cvtColor(_in, _out, cv::COLOR_RGBA2RGB);
        break;
    case asn1Sccmode_BGR:
        cv::cvtColor(_in, _out, cv::COLOR_BGR2RGB);
        break;
    case asn1Sccmode_BGRA:
        cv::cvtColor(_in, _out, cv::COLOR_BGRA2RGB);
        break;
    case asn1Sccmode_HSV:
        cv::cvtColor(_in, _out, cv::COLOR_HSV2RGB);
        break;
    case asn1Sccmode_HLS:
        cv::cvtColor(_in, _out, cv::COLOR_HLS2RGB);
        break;
    case asn1Sccmode_YUV:
        cv::cvtColor(_in, _out, cv::COLOR_YUV2RGB);
        break;
    case asn1Sccmode_Lab:
        cv::cvtColor(_in, _out, cv::COLOR_Lab2RGB);
        break;
    case asn1Sccmode_Luv:
        cv::cvtColor(_in, _out, cv::COLOR_Luv2RGB);
        break;
    case asn1Sccmode_XYZ:
        cv::cvtColor(_in, _out, cv::COLOR_XYZ2RGB);
        break;
    case asn1Sccmode_YCrCb:
        cv::cvtColor(_in, _out, cv::COLOR_YCrCb2RGB);
        break;
    case asn1Sccmode_Bayer_RGGB:
        cv::cvtColor(_in, _out, cv::COLOR_BayerRG2RGB);
        break;
    case asn1Sccmode_Bayer_GRBG:
        cv::cvtColor(_in, _out, cv::COLOR_BayerGR2RGB);
        break;
    case asn1Sccmode_Bayer_BGGR:
        cv::cvtColor(_in, _out, cv::COLOR_BayerBG2RGB);
        break;
    case asn1Sccmode_Bayer_GBRG:
        cv::cvtColor(_in, _out, cv::COLOR_BayerGB2RGB);
        break;
    default:
        std::cerr << "Bad input asn1Sccmode. Image can't be converted." << std::endl;
        break;
    }
}

void ColorConversion::convertToRGBA(){
    switch(inOriginalImage.metadata.mode){
    case asn1Sccmode_GRAY:
        cv::cvtColor(_in, _out, cv::COLOR_GRAY2RGBA);
        break;
    case asn1Sccmode_RGB:
        cv::cvtColor(_in, _out, cv::COLOR_RGB2RGBA);
        break;
    case asn1Sccmode_BGR:
        cv::cvtColor(_in, _out, cv::COLOR_BGR2RGBA);
        break;
    case asn1Sccmode_BGRA:
        cv::cvtColor(_in, _out, cv::COLOR_BGRA2RGBA);
        break;
    case asn1Sccmode_HSV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HSV2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2RGBA);
        break;
    }
    case asn1Sccmode_HLS:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HLS2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2RGBA);
        break;
    }
    case asn1Sccmode_YUV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YUV2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2RGBA);
        break;
    }
    case asn1Sccmode_Lab:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Lab2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2RGBA);
        break;
    }
    case asn1Sccmode_Luv:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Luv2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2RGBA);
        break;
    }
    case asn1Sccmode_XYZ:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_XYZ2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2RGBA);
        break;
    }
    case asn1Sccmode_YCrCb:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YCrCb2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2RGBA);
        break;
    }
    case asn1Sccmode_Bayer_RGGB:
        cv::cvtColor(_in, _out, cv::COLOR_BayerRG2RGBA);
        break;
    case asn1Sccmode_Bayer_GRBG:
        cv::cvtColor(_in, _out, cv::COLOR_BayerGR2RGBA);
        break;
    case asn1Sccmode_Bayer_BGGR:
        cv::cvtColor(_in, _out, cv::COLOR_BayerBG2RGBA);
        break;
    case asn1Sccmode_Bayer_GBRG:
        cv::cvtColor(_in, _out, cv::COLOR_BayerGB2RGBA);
        break;
    default:
        std::cerr << "Bad input asn1Sccmode. Image can't be converted." << std::endl;
        break;
    }
}

void ColorConversion::convertToBGR(){
    switch(inOriginalImage.metadata.mode){
    case asn1Sccmode_GRAY:
        cv::cvtColor(_in, _out, cv::COLOR_GRAY2BGR);
        break;
    case asn1Sccmode_RGB:
        cv::cvtColor(_in, _out, cv::COLOR_RGB2BGR);
        break;
    case asn1Sccmode_RGBA:
        cv::cvtColor(_in, _out, cv::COLOR_RGBA2BGR);
        break;
    case asn1Sccmode_BGRA:
        cv::cvtColor(_in, _out, cv::COLOR_BGRA2BGR);
        break;
    case asn1Sccmode_HSV:
        cv::cvtColor(_in, _out, cv::COLOR_HSV2BGR);
        break;
    case asn1Sccmode_HLS:
        cv::cvtColor(_in, _out, cv::COLOR_HLS2BGR);
        break;
    case asn1Sccmode_YUV:
        cv::cvtColor(_in, _out, cv::COLOR_YUV2BGR);
        break;
    case asn1Sccmode_Lab:
        cv::cvtColor(_in, _out, cv::COLOR_Lab2BGR);
        break;
    case asn1Sccmode_Luv:
        cv::cvtColor(_in, _out, cv::COLOR_Luv2BGR);
        break;
    case asn1Sccmode_XYZ:
        cv::cvtColor(_in, _out, cv::COLOR_XYZ2BGR);
        break;
    case asn1Sccmode_YCrCb:
        cv::cvtColor(_in, _out, cv::COLOR_YCrCb2BGR);
        break;
    case asn1Sccmode_Bayer_RGGB:
        cv::cvtColor(_in, _out, cv::COLOR_BayerRG2BGR);
        break;
    case asn1Sccmode_Bayer_GRBG:
        cv::cvtColor(_in, _out, cv::COLOR_BayerGR2BGR);
        break;
    case asn1Sccmode_Bayer_BGGR:
        cv::cvtColor(_in, _out, cv::COLOR_BayerBG2BGR);
        break;
    case asn1Sccmode_Bayer_GBRG:
        cv::cvtColor(_in, _out, cv::COLOR_BayerGB2BGR);
        break;
    default:
        std::cerr << "Bad input asn1Sccmode. Image can't be converted." << std::endl;
        break;
    }
}

void ColorConversion::convertToBGRA(){
    switch(inOriginalImage.metadata.mode){
    case asn1Sccmode_GRAY:
        cv::cvtColor(_in, _out, cv::COLOR_GRAY2BGRA);
        break;
    case asn1Sccmode_RGB:
        cv::cvtColor(_in, _out, cv::COLOR_RGB2BGRA);
        break;
    case asn1Sccmode_RGBA:
        cv::cvtColor(_in, _out, cv::COLOR_RGBA2BGRA);
        break;
    case asn1Sccmode_BGR:
        cv::cvtColor(_in, _out, cv::COLOR_BGR2BGRA);
        break;
    case asn1Sccmode_HSV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HSV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2BGRA);
        break;
    }
    case asn1Sccmode_HLS:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HLS2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2BGRA);
        break;
    }
    case asn1Sccmode_YUV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YUV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2BGRA);
        break;
    }
    case asn1Sccmode_Lab:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Lab2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2BGRA);
        break;
    }
    case asn1Sccmode_Luv:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Luv2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2BGRA);
        break;
    }
    case asn1Sccmode_XYZ:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_XYZ2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2BGRA);
        break;
    }
    case asn1Sccmode_YCrCb:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YCrCb2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2BGRA);
        break;
    }
    case asn1Sccmode_Bayer_RGGB:
        cv::cvtColor(_in, _out, cv::COLOR_BayerRG2BGRA);
        break;
    case asn1Sccmode_Bayer_GRBG:
        cv::cvtColor(_in, _out, cv::COLOR_BayerGR2BGRA);
        break;
    case asn1Sccmode_Bayer_BGGR:
        cv::cvtColor(_in, _out, cv::COLOR_BayerBG2BGRA);
        break;
    case asn1Sccmode_Bayer_GBRG:
        cv::cvtColor(_in, _out, cv::COLOR_BayerGB2BGRA);
        break;
    default:
        std::cerr << "Bad input asn1Sccmode. Image can't be converted." << std::endl;
        break;
    }
}

void ColorConversion::convertToHSV(){
    switch(inOriginalImage.metadata.mode){
    case asn1Sccmode_GRAY:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_GRAY2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HSV);
        break;
    }
    case asn1Sccmode_RGB:
        cv::cvtColor(_in, _out, cv::COLOR_RGB2HSV);
        break;
    case asn1Sccmode_RGBA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_RGBA2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2HSV);
        break;
    }
    case asn1Sccmode_BGR:
        cv::cvtColor(_in, _out, cv::COLOR_BGR2HSV);
        break;
    case asn1Sccmode_BGRA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BGRA2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HSV);
        break;
    }
    case asn1Sccmode_HLS:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HLS2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HSV);
        break;
    }
    case asn1Sccmode_YUV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YUV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HSV);
        break;
    }
    case asn1Sccmode_Lab:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Lab2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HSV);
        break;
    }
    case asn1Sccmode_Luv:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Luv2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HSV);
        break;
    }
    case asn1Sccmode_XYZ:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_XYZ2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HSV);
        break;
    }
    case asn1Sccmode_YCrCb:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YCrCb2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HSV);
        break;
    }
    case asn1Sccmode_Bayer_RGGB:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HSV);
        break;
    }
    case asn1Sccmode_Bayer_GRBG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGR2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HSV);
        break;
    }
    case asn1Sccmode_Bayer_BGGR:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HSV);
        break;
    }
    case asn1Sccmode_Bayer_GBRG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGB2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HSV);
        break;
    }
    default:
        std::cerr << "Bad input asn1Sccmode. Image can't be converted." << std::endl;
        break;
    }
}

void ColorConversion::convertToHLS(){
    switch(inOriginalImage.metadata.mode){
    case asn1Sccmode_GRAY:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_GRAY2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HLS);
        break;
    }
    case asn1Sccmode_RGB:
        cv::cvtColor(_in, _out, cv::COLOR_RGB2HLS);
        break;
    case asn1Sccmode_RGBA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_RGBA2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2HLS);
        break;
    }
    case asn1Sccmode_BGR:
        cv::cvtColor(_in, _out, cv::COLOR_BGR2HLS);
        break;
    case asn1Sccmode_BGRA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BGRA2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HLS);
        break;
    }
    case asn1Sccmode_HSV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HSV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HLS);
        break;
    }
    case asn1Sccmode_YUV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YUV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HLS);
        break;
    }
    case asn1Sccmode_Lab:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Lab2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HLS);
        break;
    }
    case asn1Sccmode_Luv:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Luv2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HLS);
        break;
    }
    case asn1Sccmode_XYZ:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_XYZ2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HLS);
        break;
    }
    case asn1Sccmode_YCrCb:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YCrCb2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HLS);
        break;
    }
    case asn1Sccmode_Bayer_RGGB:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HLS);
        break;
    }
    case asn1Sccmode_Bayer_GRBG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGR2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HLS);
        break;
    }
    case asn1Sccmode_Bayer_BGGR:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HLS);
        break;
    }
    case asn1Sccmode_Bayer_GBRG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGB2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2HLS);
        break;
    }
    default:
        std::cerr << "Bad input asn1Sccmode. Image can't be converted." << std::endl;
        break;
    }
}

void ColorConversion::convertToYUV(){
    switch(inOriginalImage.metadata.mode){
    case asn1Sccmode_GRAY:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_GRAY2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YUV);
        break;
    }
    case asn1Sccmode_RGB:
        cv::cvtColor(_in, _out, cv::COLOR_RGB2YUV);
        break;
    case asn1Sccmode_RGBA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_RGBA2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2YUV);
        break;
    }
    case asn1Sccmode_BGR:
        cv::cvtColor(_in, _out, cv::COLOR_BGR2YUV);
        break;
    case asn1Sccmode_BGRA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BGRA2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YUV);
        break;
    }
    case asn1Sccmode_HSV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HSV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YUV);
        break;
    }
    case asn1Sccmode_HLS:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HLS2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YUV);
        break;
    }
    case asn1Sccmode_Lab:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Lab2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YUV);
        break;
    }
    case asn1Sccmode_Luv:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Luv2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YUV);
        break;
    }
    case asn1Sccmode_XYZ:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_XYZ2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YUV);
        break;
    }
    case asn1Sccmode_YCrCb:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YCrCb2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YUV);
        break;
    }
    case asn1Sccmode_Bayer_RGGB:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YUV);
        break;
    }
    case asn1Sccmode_Bayer_GRBG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGR2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YUV);
        break;
    }
    case asn1Sccmode_Bayer_BGGR:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YUV);
        break;
    }
    case asn1Sccmode_Bayer_GBRG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGB2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YUV);
        break;
    }
    default:
        std::cerr << "Bad input asn1Sccmode. Image can't be converted." << std::endl;
        break;
    }
}

void ColorConversion::convertToLab(){
    switch(inOriginalImage.metadata.mode){
    case asn1Sccmode_GRAY:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_GRAY2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Lab);
        break;
    }
    case asn1Sccmode_RGB:
        cv::cvtColor(_in, _out, cv::COLOR_RGB2Lab);
        break;
    case asn1Sccmode_RGBA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_RGBA2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2Lab);
        break;
    }
    case asn1Sccmode_BGR:
        cv::cvtColor(_in, _out, cv::COLOR_BGR2Lab);
        break;
    case asn1Sccmode_BGRA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BGRA2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Lab);
        break;
    }
    case asn1Sccmode_HSV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HSV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Lab);
        break;
    }
    case asn1Sccmode_HLS:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HLS2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Lab);
        break;
    }
    case asn1Sccmode_YUV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YUV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Lab);
        break;
    }
    case asn1Sccmode_Luv:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Luv2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Lab);
        break;
    }
    case asn1Sccmode_XYZ:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_XYZ2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Lab);
        break;
    }
    case asn1Sccmode_YCrCb:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YCrCb2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Lab);
        break;
    }
    case asn1Sccmode_Bayer_RGGB:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Lab);
        break;
    }
    case asn1Sccmode_Bayer_GRBG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGR2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Lab);
        break;
    }
    case asn1Sccmode_Bayer_BGGR:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Lab);
        break;
    }
    case asn1Sccmode_Bayer_GBRG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGB2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Lab);
        break;
    }
    default:
        std::cerr << "Bad input asn1Sccmode. Image can't be converted." << std::endl;
        break;
    }
}

void ColorConversion::convertToLuv(){
    switch(inOriginalImage.metadata.mode){
    case asn1Sccmode_GRAY:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_GRAY2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Luv);
        break;
    }
    case asn1Sccmode_RGB:
        cv::cvtColor(_in, _out, cv::COLOR_RGB2Luv);
        break;
    case asn1Sccmode_RGBA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_RGBA2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2Luv);
        break;
    }
    case asn1Sccmode_BGR:
        cv::cvtColor(_in, _out, cv::COLOR_BGR2Luv);
        break;
    case asn1Sccmode_BGRA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BGRA2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Luv);
        break;
    }
    case asn1Sccmode_HSV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HSV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Luv);
        break;
    }
    case asn1Sccmode_HLS:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HLS2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Luv);
        break;
    }
    case asn1Sccmode_YUV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YUV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Luv);
        break;
    }
    case asn1Sccmode_Lab:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Lab2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Luv);
        break;
    }
    case asn1Sccmode_XYZ:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_XYZ2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Luv);
        break;
    }
    case asn1Sccmode_YCrCb:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YCrCb2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Luv);
        break;
    }
    case asn1Sccmode_Bayer_RGGB:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Luv);
        break;
    }
    case asn1Sccmode_Bayer_GRBG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGR2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Luv);
        break;
    }
    case asn1Sccmode_Bayer_BGGR:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Luv);
        break;
    }
    case asn1Sccmode_Bayer_GBRG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGB2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2Luv);
        break;
    }
    default:
        std::cerr << "Bad input asn1Sccmode. Image can't be converted." << std::endl;
        break;
    }
}

void ColorConversion::convertToXYZ(){
    switch(inOriginalImage.metadata.mode){
    case asn1Sccmode_GRAY:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_GRAY2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2XYZ);
        break;
    }
    case asn1Sccmode_RGB:
        cv::cvtColor(_in, _out, cv::COLOR_RGB2XYZ);
        break;
    case asn1Sccmode_RGBA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_RGBA2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2XYZ);
        break;
    }
    case asn1Sccmode_BGR:
        cv::cvtColor(_in, _out, cv::COLOR_BGR2XYZ);
        break;
    case asn1Sccmode_BGRA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BGRA2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2XYZ);
        break;
    }
    case asn1Sccmode_HSV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HSV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2XYZ);
        break;
    }
    case asn1Sccmode_HLS:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HLS2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2XYZ);
        break;
    }
    case asn1Sccmode_YUV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YUV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2XYZ);
        break;
    }
    case asn1Sccmode_Lab:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Lab2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2XYZ);
        break;
    }
    case asn1Sccmode_Luv:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Luv2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2XYZ);
        break;
    }
    case asn1Sccmode_YCrCb:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YCrCb2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2XYZ);
        break;
    }
    case asn1Sccmode_Bayer_RGGB:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2XYZ);
        break;
    }
    case asn1Sccmode_Bayer_GRBG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGR2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2XYZ);
        break;
    }
    case asn1Sccmode_Bayer_BGGR:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2XYZ);
        break;
    }
    case asn1Sccmode_Bayer_GBRG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGB2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2XYZ);
        break;
    }
    default:
        std::cerr << "Bad input asn1Sccmode. Image can't be converted." << std::endl;
        break;
    }
}

void ColorConversion::convertToYCrCb(){
    switch(inOriginalImage.metadata.mode){
    case asn1Sccmode_GRAY:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_GRAY2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YCrCb);
        break;
    }
    case asn1Sccmode_RGB:
        cv::cvtColor(_in, _out, cv::COLOR_RGB2YCrCb);
        break;
    case asn1Sccmode_RGBA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_RGBA2RGB);
        cv::cvtColor(tmp, _out, cv::COLOR_RGB2YCrCb);
        break;
    }
    case asn1Sccmode_BGR:
        cv::cvtColor(_in, _out, cv::COLOR_BGR2YCrCb);
        break;
    case asn1Sccmode_BGRA:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BGRA2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YCrCb);
        break;
    }
    case asn1Sccmode_HSV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HSV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YCrCb);
        break;
    }
    case asn1Sccmode_HLS:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_HLS2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YCrCb);
        break;
    }
    case asn1Sccmode_YUV:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_YUV2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YCrCb);
        break;
    }
    case asn1Sccmode_Lab:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Lab2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YCrCb);
        break;
    }
    case asn1Sccmode_Luv:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_Luv2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YCrCb);
        break;
    }
    case asn1Sccmode_XYZ:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_XYZ2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YCrCb);
        break;
    }
    case asn1Sccmode_Bayer_RGGB:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerRG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YCrCb);
        break;
    }
    case asn1Sccmode_Bayer_GRBG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGR2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YCrCb);
        break;
    }
    case asn1Sccmode_Bayer_BGGR:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerBG2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YCrCb);
        break;
    }
    case asn1Sccmode_Bayer_GBRG:{
        cv::Mat tmp;
        cv::cvtColor(_in, tmp, cv::COLOR_BayerGB2BGR);
        cv::cvtColor(tmp, _out, cv::COLOR_BGR2YCrCb);
        break;
    }
    default:
        std::cerr << "Bad input asn1Sccmode. Image can't be converted." << std::endl;
        break;
    }
}

void ColorConversion::ValidateParameters()
{
    ASSERT(parameters.targetMode >= 1 && parameters.targetMode <= 13 && parameters.targetMode != 9, "targetMode has to be within [1..8]U[10..13]");
}

const ColorConversion::ColorConversionParams ColorConversion::DEFAULT_PARAMETERS = {
    .targetMode = 1
};

}
}
}

/** @} */
