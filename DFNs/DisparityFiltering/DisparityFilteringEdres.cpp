/**
 * @addtogroup DFNs
 * @{
 */

#include "DisparityFilteringEdres.hpp"
#include "edres-wrapper/EdresStereo.h"

namespace CDFF
{
namespace DFN
{
namespace DisparityFiltering
{

DisparityFilteringEdres::DisparityFilteringEdres()
{
    parametersHelper.AddParameter<int>("DisparityFilteringEdresParams", "trimWidth", parameters.trimWidth, DEFAULT_PARAMETERS.trimWidth);
    parametersHelper.AddParameter<float>("DisparityFilteringEdresParams", "connexityThresh", parameters.connexityThresh, DEFAULT_PARAMETERS.connexityThresh);
    parametersHelper.AddParameter<int>("DisparityFilteringEdresParams", "surfMin", parameters.surfMin, DEFAULT_PARAMETERS.surfMin);
    parametersHelper.AddParameter<int>("DisparityFilteringEdresParams", "surfMax", parameters.surfMax, DEFAULT_PARAMETERS.surfMax);

    configurationFilePath = "";
}

DisparityFilteringEdres::~DisparityFilteringEdres()
{
}

void DisparityFilteringEdres::configure()
{
    if(configurationFilePath != ""){
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void DisparityFilteringEdres::process()
{
    Edres::disparitiesFiltering(inRawDisparity, outFilteredDisparity, parameters.trimWidth, parameters.connexityThresh, parameters.surfMin, parameters.surfMax);
}

void DisparityFilteringEdres::ValidateParameters()
{
    ASSERT(parameters.trimWidth > 0, "trimWidth has to be positive");
    ASSERT(parameters.connexityThresh >= 0, "connexityThresh has to be positive");
    ASSERT(parameters.surfMin >= 0, "surfMin has to be positive");
    ASSERT(parameters.surfMax >= 0 && parameters.surfMax > parameters.surfMin, "surfMax has to be positive and > surfMin");
}

const DisparityFilteringEdres::DisparityFilteringEdresParams DisparityFilteringEdres::DEFAULT_PARAMETERS = {
    .trimWidth = 3,
    .connexityThresh = 2.0,
    .surfMin = 20,
    .surfMax = 1000
};

}
}
}

/** @} */
