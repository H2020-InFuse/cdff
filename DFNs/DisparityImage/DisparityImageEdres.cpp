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
    parametersHelper.AddParameter<double>("DisparityParams", "minDistance", parameters.disparityParams.minDistance, DEFAULT_PARAMETERS.disparityParams.minDistance);
    parametersHelper.AddParameter<double>("DisparityParams", "maxDistance", parameters.disparityParams.maxDistance, DEFAULT_PARAMETERS.disparityParams.maxDistance);
    parametersHelper.AddParameter<int>("DisparityParams", "method", parameters.disparityParams.method, DEFAULT_PARAMETERS.disparityParams.method);
    parametersHelper.AddParameter<int>("DisparityParams", "grad", parameters.disparityParams.grad, DEFAULT_PARAMETERS.disparityParams.grad);
    parametersHelper.AddParameter<int>("DisparityParams", "gradType", parameters.disparityParams.gradType, DEFAULT_PARAMETERS.disparityParams.gradType);
    parametersHelper.AddParameter<int>("DisparityParams", "dispType", parameters.disparityParams.dispType, DEFAULT_PARAMETERS.disparityParams.dispType);

    parametersHelper.AddParameter<bool>("FilterParams", "filter", parameters.filterParams.filter, DEFAULT_PARAMETERS.filterParams.filter);
    parametersHelper.AddParameter<int>("FilterParams", "trimWidth", parameters.filterParams.trimWidth, DEFAULT_PARAMETERS.filterParams.trimWidth);
    parametersHelper.AddParameter<float>("FilterParams", "connexityThresh", parameters.filterParams.connexityThresh, DEFAULT_PARAMETERS.filterParams.connexityThresh);
    parametersHelper.AddParameter<int>("FilterParams", "surfMin", parameters.filterParams.surfMin, DEFAULT_PARAMETERS.filterParams.surfMin);
    parametersHelper.AddParameter<int>("FilterParams", "surfMax", parameters.filterParams.surfMax, DEFAULT_PARAMETERS.filterParams.surfMax);

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
    Edres::disparities(inFramePair, outDisparity, parameters.disparityParams.minDistance, parameters.disparityParams.maxDistance, static_cast<Edres::StereoMethod>(parameters.disparityParams.method), static_cast<Edres::Gradient>(parameters.disparityParams.grad), static_cast<Edres::PixelDepth>(parameters.disparityParams.gradType), static_cast<Edres::PixelDepth>(parameters.disparityParams.dispType));

    if(parameters.filterParams.filter){
        Edres::disparitiesFiltering(outDisparity, outDisparity, parameters.filterParams.trimWidth, parameters.filterParams.connexityThresh, parameters.filterParams.surfMin, parameters.filterParams.surfMax);
    }
}

void DisparityImageEdres::ValidateParameters()
{
    ASSERT(parameters.disparityParams.minDistance >= 0, "minDistance has to be positive");
    ASSERT(parameters.disparityParams.maxDistance > 0 && parameters.disparityParams.maxDistance > parameters.disparityParams.minDistance, "maxDistance has to be positive and >= minDistance");
    ASSERT(parameters.disparityParams.method >= 0 && parameters.disparityParams.method <= 4, "method has to be within [0..4]");
    ASSERT(parameters.disparityParams.grad >= 0 && parameters.disparityParams.grad <= 5, "grad has to be within [0..5]");
    ASSERT(parameters.disparityParams.gradType >= 5 && parameters.disparityParams.gradType <= 8, "gradType has to be within [5..8]");
    ASSERT(parameters.disparityParams.dispType >= 5 && parameters.disparityParams.dispType <= 8, "dispType has to be within [5..8]");

    ASSERT(parameters.filterParams.trimWidth >= 0, "trimWidth has to be positive");
    ASSERT(parameters.filterParams.connexityThresh >= 0, "connexityThresh has to be positive");
    ASSERT(parameters.filterParams.surfMin >= 0, "surfMin has to be positive");
    ASSERT(parameters.filterParams.surfMax >= 0 && parameters.filterParams.surfMax > parameters.filterParams.surfMin, "surfMax has to be positive and > surfMin");
}

const DisparityImageEdres::DisparityImageEdresParams DisparityImageEdres::DEFAULT_PARAMETERS = {
    {
        .minDistance = 1.0,
        .maxDistance = 40.0,
        .method = Edres::GRADLINE,
        .grad = Edres::LAPGAUS99,
        .gradType = Edres::BYTE,
        .dispType = Edres::FLOAT
    },
    {
        .filter = true,
        .trimWidth = 3,
        .connexityThresh = 2.0,
        .surfMin = 20,
        .surfMax = 1000
    }
};

}
}
}

/** @} */
