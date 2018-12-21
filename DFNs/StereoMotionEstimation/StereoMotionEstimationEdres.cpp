/**
 * @addtogroup DFNs
 * @{
 */

#include "StereoMotionEstimationEdres.hpp"
#include <iostream>

namespace CDFF
{
namespace DFN
{
namespace StereoMotionEstimation
{

StereoMotionEstimationEdres::StereoMotionEstimationEdres()
{
    _vo = new Edres::VisualOdometry();

    parametersHelper.AddParameter<int>("StereoMotionEstimationEdresParams", "method", parameters.method, DEFAULT_PARAMETERS.method);
    parametersHelper.AddParameter<int>("StereoMotionEstimationEdresParams", "motionEstimationMatchingAlgo", parameters.motionEstimationMatchingAlgo, DEFAULT_PARAMETERS.motionEstimationMatchingAlgo);
    parametersHelper.AddParameter<int>("StereoMotionEstimationEdresParams", "featureMatchingAlgo", parameters.featureMatchingAlgo, DEFAULT_PARAMETERS.featureMatchingAlgo);
    parametersHelper.AddParameter<int>("StereoMotionEstimationEdresParams", "bundleAdjustmentType", parameters.bundleAdjustmentType, DEFAULT_PARAMETERS.bundleAdjustmentType);
    parametersHelper.AddParameter<bool>("StereoMotionEstimationEdresParams", "openLoop", parameters.openLoop, DEFAULT_PARAMETERS.openLoop);
    parametersHelper.AddParameter<int>("StereoMotionEstimationEdresParams", "theshold_us", parameters.theshold_us, DEFAULT_PARAMETERS.theshold_us);

    configurationFilePath = "";
}

StereoMotionEstimationEdres::~StereoMotionEstimationEdres()
{
    if(_vo){
        delete _vo;
        _vo = nullptr;
    }
}

void StereoMotionEstimationEdres::configure()
{
    if(configurationFilePath != ""){
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();

    _vo->create();
}

void StereoMotionEstimationEdres::process()
{
    if( std::fabs(inFramePair.left.metadata.timeStamp.microseconds - inDisparity.metadata.timeStamp.microseconds) <= parameters.theshold_us ){
        _vo->getParams()->_method = parameters.method;
        _vo->getParams()->_matchProcess = parameters.motionEstimationMatchingAlgo;
        _vo->getParams()->_matchingAlgo = parameters.featureMatchingAlgo;
        _vo->getParams()->_baType = parameters.bundleAdjustmentType;

        if(parameters.openLoop){
            _vo->getParams()->_vmeIn = true;
            _vo->getParams()->_forceInIdentity = true;
            _vo->getParams()->_forceNoGps = true;
        }
        else{
            _vo->getParams()->_vmeIn = false;
            _vo->getParams()->_forceInIdentity = false;
            _vo->getParams()->_forceNoGps = false;
        }

        _vo->updateParams();
        _vo->addNewAcquisition(inFramePair, inDisparity);
        _vo->process();
        outPose = _vo->getPose();
    }
    else{
        std::cerr << "StereoMotionEstimationEdres: Input Frame pair and disparity image are not synchronized." << std::endl;
    }
}

void StereoMotionEstimationEdres::ValidateParameters()
{
    ASSERT(parameters.method >= 0 && parameters.method <= 1, "method has to be within [0..1]");
    ASSERT(parameters.motionEstimationMatchingAlgo >= 0 && parameters.motionEstimationMatchingAlgo <= 23, "motionEstimationMatchingAlgo has to be within [0..23]");
    ASSERT(parameters.featureMatchingAlgo >= 0 && parameters.featureMatchingAlgo <= 1, "featureMatchingAlgo has to be within [0..23]");
    ASSERT(parameters.bundleAdjustmentType >= 0 && parameters.bundleAdjustmentType <= 2, "bundleAdjustmentType has to be within [0..2]");
    ASSERT(parameters.theshold_us >= 0, "theshold_us has to be >= 0");
}

const StereoMotionEstimationEdres::StereoMotionEstimationEdresParams StereoMotionEstimationEdres::DEFAULT_PARAMETERS = {
    .method = 0,
    .motionEstimationMatchingAlgo = 11,
    .featureMatchingAlgo = 7,
    .bundleAdjustmentType = 0,
    .openLoop = false,
    .theshold_us = 20
};

}
}
}

/** @} */
