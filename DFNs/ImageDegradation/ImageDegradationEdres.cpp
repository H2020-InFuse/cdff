/**
 * @addtogroup DFNs
 * @{
 */

#include "ImageDegradationEdres.hpp"
#include "edres-wrapper/EdresRectification.h"

namespace CDFF
{
namespace DFN
{
namespace ImageDegradation
{

ImageDegradationEdres::ImageDegradationEdres()
{
    parametersHelper.AddParameter<int>("ImageDegradationEdresParams", "xratio", parameters.xratio, DEFAULT_PARAMETERS.xratio);
    parametersHelper.AddParameter<int>("ImageDegradationEdresParams", "yratio", parameters.yratio, DEFAULT_PARAMETERS.yratio);
    parametersHelper.AddParameter<int>("ImageDegradationEdresParams", "method", parameters.method, DEFAULT_PARAMETERS.method);
    parametersHelper.AddParameter<int>("ImageDegradationEdresParams", "outType", parameters.outType, DEFAULT_PARAMETERS.outType);

    configurationFilePath = "";
}

ImageDegradationEdres::~ImageDegradationEdres()
{
}

void ImageDegradationEdres::configure()
{
    if(configurationFilePath != ""){
      parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void ImageDegradationEdres::process()
{
    Edres::degradation(inOriginalImage, outDegradedImage, parameters.xratio, parameters.yratio, static_cast<Edres::DegradationMethod>(parameters.method), static_cast<Edres::PixelDepth>(parameters.outType));
}

void ImageDegradationEdres::ValidateParameters()
{
    ASSERT(parameters.xratio >= 1 && parameters.xratio <= 25, "xratio has to be within [1..25]");
    ASSERT(parameters.yratio >= 1 && parameters.yratio <= 25, "yratio has to be within [1..25]");
    ASSERT(parameters.method == 0 || parameters.method == 1, "method has to be 0 or 1");
    ASSERT(parameters.outType >= 5 && parameters.outType <= 8, "outType has to be within [5..8]");
}

const ImageDegradationEdres::ImageDegradationEdresParams ImageDegradationEdres::DEFAULT_PARAMETERS = {
    .xratio = 2,
    .yratio = 2,
    .method = 0,
    .outType = 5
};

}
}
}

/** @} */
