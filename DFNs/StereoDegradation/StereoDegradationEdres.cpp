/**
 * @addtogroup DFNs
 * @{
 */

#include "StereoDegradationEdres.hpp"
#include "edres-wrapper/EdresRectification.h"

namespace CDFF
{
namespace DFN
{
namespace StereoDegradation
{

StereoDegradationEdres::StereoDegradationEdres()
:parameters(DEFAULT_PARAMETERS)
{
    parametersHelper.AddParameter<int>("StereoDegradationEdresParams", "xratio", parameters.xratio, DEFAULT_PARAMETERS.xratio);
    parametersHelper.AddParameter<int>("StereoDegradationEdresParams", "yratio", parameters.yratio, DEFAULT_PARAMETERS.yratio);
    parametersHelper.AddParameter<int>("StereoDegradationEdresParams", "method", parameters.method, DEFAULT_PARAMETERS.method);
    parametersHelper.AddParameter<int>("StereoDegradationEdresParams", "outType", parameters.outType, DEFAULT_PARAMETERS.outType);

    configurationFilePath = "";
}

StereoDegradationEdres::~StereoDegradationEdres()
{
}

void StereoDegradationEdres::configure()
{
    if(configurationFilePath != ""){
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void StereoDegradationEdres::process()
{
    Edres::degradation(inOriginalImagePair, outDegradedImagePair, parameters.xratio, parameters.yratio, static_cast<Edres::DegradationMethod>(parameters.method), static_cast<Edres::PixelDepth>(parameters.outType));
}

void StereoDegradationEdres::ValidateParameters()
{
    ASSERT(parameters.xratio >= 1 && parameters.xratio <= 25, "xratio has to be within [1..25]");
    ASSERT(parameters.yratio >= 1 && parameters.yratio <= 25, "yratio has to be within [1..25]");
    ASSERT(parameters.method == 0 || parameters.method == 1, "method has to be 0 or 1");
    ASSERT(parameters.outType >= 5 && parameters.outType <= 8, "outType has to be within [5..8]");
}

const StereoDegradationEdres::StereoDegradationEdresParams StereoDegradationEdres::DEFAULT_PARAMETERS = {
    .xratio = 2,
    .yratio = 2,
    .method = 0,
    .outType = 5
};

}
}
}

/** @} */
