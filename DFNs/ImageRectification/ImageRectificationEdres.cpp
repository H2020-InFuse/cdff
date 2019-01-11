/**
 * @addtogroup DFNs
 * @{
 */

#include "ImageRectificationEdres.hpp"

namespace CDFF
{
namespace DFN
{
namespace ImageRectification
{

ImageRectificationEdres::ImageRectificationEdres()
:parameters(DEFAULT_PARAMETERS)
{
    _mapFile = " ";
    _rectification = new Edres::Rectification();
    _initialized = false;

    parametersHelper.AddParameter<std::string>("ImageRectificationEdresParams", "mapFile", parameters.mapFile, DEFAULT_PARAMETERS.mapFile);
    parametersHelper.AddParameter<int>("ImageRectificationEdresParams", "xratio", parameters.xratio, DEFAULT_PARAMETERS.xratio);
    parametersHelper.AddParameter<int>("ImageRectificationEdresParams", "yratio", parameters.yratio, DEFAULT_PARAMETERS.yratio);
    parametersHelper.AddParameter<int>("ImageRectificationEdresParams", "xshift", parameters.xshift, DEFAULT_PARAMETERS.xshift);
    parametersHelper.AddParameter<int>("ImageRectificationEdresParams", "yshift", parameters.yshift, DEFAULT_PARAMETERS.yshift);
    parametersHelper.AddParameter<int>("ImageRectificationEdresParams", "outType", parameters.outType, DEFAULT_PARAMETERS.outType);

    configurationFilePath = "";
}

ImageRectificationEdres::~ImageRectificationEdres()
{
    if( _rectification ){
        delete _rectification;
        _rectification = nullptr;
    }
}

void ImageRectificationEdres::configure()
{
    if(configurationFilePath != ""){
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void ImageRectificationEdres::process()
{
    if( _mapFile != parameters.mapFile ){
        _mapFile = parameters.mapFile;

        if( !_rectification->init(inOriginalImage.data.cols, inOriginalImage.data.rows, _mapFile)){
            _initialized = true;
        }
        else{
            _initialized = false;
        }
    }

    if(_initialized){
        (*_rectification)(inOriginalImage, outRectifiedImage, parameters.xratio, parameters.yratio, static_cast<Edres::PixelDepth>(parameters.outType), parameters.xshift, parameters.yshift);
    }
}

void ImageRectificationEdres::ValidateParameters()
{
    ASSERT(parameters.xratio >= 1 && parameters.xratio <= 25, "xratio has to be within [1..25]");
    ASSERT(parameters.yratio >= 1 && parameters.yratio <= 25, "yratio has to be within [1..25]");
    ASSERT(parameters.outType >= 5 && parameters.outType <= 8, "outType has to be within [5..8]");
}

const ImageRectificationEdres::ImageRectificationEdresParams ImageRectificationEdres::DEFAULT_PARAMETERS = {
    .mapFile = "",
    .xratio = 1,
    .yratio = 1,
    .xshift = 0,
    .yshift = 0,
    .outType = 5
};

}
}
}

/** @} */
