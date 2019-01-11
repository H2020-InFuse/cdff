/**
 * @addtogroup DFNs
 * @{
 */

#include "DisparityImageEdres.hpp"
#include "edres-wrapper/EdresStereo.h"

namespace CDFF
{
namespace DFN
{
namespace DisparityImage
{

DisparityImageEdres::DisparityImageEdres()
:parameters(DEFAULT_PARAMETERS)
{
    parametersHelper.AddParameter<double>("DisparityImageEdresParams", "minDistance", parameters.minDistance, DEFAULT_PARAMETERS.minDistance);
    parametersHelper.AddParameter<double>("DisparityImageEdresParams", "maxDistance", parameters.maxDistance, DEFAULT_PARAMETERS.maxDistance);
    parametersHelper.AddParameter<int>("DisparityImageEdresParams", "method", parameters.method, DEFAULT_PARAMETERS.method);
    parametersHelper.AddParameter<int>("DisparityImageEdresParams", "grad", parameters.grad, DEFAULT_PARAMETERS.grad);
    parametersHelper.AddParameter<int>("DisparityImageEdresParams", "gradType", parameters.gradType, DEFAULT_PARAMETERS.gradType);
    parametersHelper.AddParameter<int>("DisparityImageEdresParams", "dispType", parameters.dispType, DEFAULT_PARAMETERS.dispType);

    configurationFilePath = "";
}

DisparityImageEdres::~DisparityImageEdres()
{
}

void DisparityImageEdres::configure()
{
    if(configurationFilePath!="")
    {
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void DisparityImageEdres::process()
{
    Edres::disparities(inFramePair, outRawDisparity, parameters.minDistance, parameters.maxDistance, static_cast<Edres::StereoMethod>(parameters.method), static_cast<Edres::Gradient>(parameters.grad), static_cast<Edres::PixelDepth>(parameters.gradType), static_cast<Edres::PixelDepth>(parameters.dispType));
}

void DisparityImageEdres::ValidateParameters()
{
    ASSERT(parameters.minDistance >= 0, "minDistance has to be positive");
    ASSERT(parameters.maxDistance > 0 && parameters.maxDistance > parameters.minDistance, "maxDistance has to be positive and >= minDistance");
    ASSERT(parameters.method >= 0 && parameters.method <= 4, "method has to be within [0..4]");
    ASSERT(parameters.grad >= 0 && parameters.grad <= 5, "grad has to be within [0..5]");
    ASSERT(parameters.gradType >= 5 && parameters.gradType <= 8, "gradType has to be within [5..8]");
    ASSERT(parameters.dispType >= 5 && parameters.dispType <= 8, "dispType has to be within [5..8]");
}

const DisparityImageEdres::DisparityImageEdresParams DisparityImageEdres::DEFAULT_PARAMETERS = {
    .minDistance = 1.0,
    .maxDistance = 40.0,
    .method = Edres::GRADLINE,
    .grad = Edres::LAPGAUS99,
    .gradType = Edres::BYTE,
    .dispType = Edres::FLOAT};
}
}
}

/** @} */
