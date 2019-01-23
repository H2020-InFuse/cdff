/**
 * @addtogroup DFNs
 * @{
 */

#include "StereoRectificationEdres.hpp"

namespace CDFF
{
namespace DFN
{
namespace StereoRectification
{

StereoRectificationEdres::StereoRectificationEdres()
:parameters(DEFAULT_PARAMETERS)
{
    _mapFileLeft = " ";
    _mapFileRight = " ";
    _rectification = new Edres::StereoRectification();
    _initialized = false;

    parametersHelper.AddParameter<std::string>("StereoRectificationEdresParams", "mapFileLeft", parameters.mapFileLeft, DEFAULT_PARAMETERS.mapFileLeft);
    parametersHelper.AddParameter<std::string>("StereoRectificationEdresParams", "mapFileRight", parameters.mapFileRight, DEFAULT_PARAMETERS.mapFileRight);
    parametersHelper.AddParameter<int>("StereoRectificationEdresParams", "xratio", parameters.xratio, DEFAULT_PARAMETERS.xratio);
    parametersHelper.AddParameter<int>("StereoRectificationEdresParams", "yratio", parameters.yratio, DEFAULT_PARAMETERS.yratio);
    parametersHelper.AddParameter<int>("StereoRectificationEdresParams", "outType", parameters.outType, DEFAULT_PARAMETERS.outType);

    configurationFilePath = "";
}

StereoRectificationEdres::~StereoRectificationEdres()
{
    if( _rectification ){
        delete _rectification;
        _rectification = nullptr;
    }
}

void StereoRectificationEdres::configure()
{
    if(configurationFilePath != ""){
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void StereoRectificationEdres::process()
{
    if( _mapFileLeft != parameters.mapFileLeft ||
            _mapFileRight != parameters.mapFileRight ){
        _mapFileLeft = parameters.mapFileLeft;
        _mapFileRight = parameters.mapFileRight;

        if( !_rectification->init(inOriginalStereoPair.left.data.cols, inOriginalStereoPair.left.data.rows, _mapFileLeft, _mapFileRight)){
            _initialized = true;
        }
        else{
            _initialized = false;
        }
    }

    if(_initialized){
        (*_rectification)(inOriginalStereoPair, outRectifiedStereoPair, parameters.xratio, parameters.yratio, static_cast<Edres::PixelDepth>(parameters.outType));
    }
}

void StereoRectificationEdres::ValidateParameters()
{
    ASSERT(parameters.xratio >= 1 && parameters.xratio <= 25, "xratio has to be within [1..25]");
    ASSERT(parameters.yratio >= 1 && parameters.yratio <= 25, "yratio has to be within [1..25]");
    ASSERT(parameters.outType >= 5 && parameters.outType <= 8, "outType has to be within [5..8]");
}

const StereoRectificationEdres::StereoRectificationEdresParams StereoRectificationEdres::DEFAULT_PARAMETERS = {
    .mapFileLeft = "",
    .mapFileRight = "",
    .xratio = 1,
    .yratio = 1,
    .outType = 5
};

}
}
}

/** @} */
