/**
 * @addtogroup DFNs
 * @{
 */

#include "LinemodTrain.hpp"
#include "linemod-wrapper/LinemodTrainImpl.hpp"

namespace CDFF
{
namespace DFN
{
namespace LinemodTrain
{

LinemodTrain::LinemodTrain()
{
    parameters = DEFAULT_PARAMETERS;

    parametersHelper.AddParameter<int>("LinemodTrainParams", "T_level0", parameters.T_level0, DEFAULT_PARAMETERS.T_level0);
    parametersHelper.AddParameter<int>("LinemodTrainParams", "T_level1", parameters.T_level1, DEFAULT_PARAMETERS.T_level1);
    parametersHelper.AddParameter<int>("LinemodTrainParams", "renderWindowWidth", parameters.renderWindowWidth, DEFAULT_PARAMETERS.renderWindowWidth);
    parametersHelper.AddParameter<int>("LinemodTrainParams", "renderWindowWidth", parameters.renderWindowWidth, DEFAULT_PARAMETERS.renderWindowWidth);
    parametersHelper.AddParameter<int>("LinemodTrainParams", "renderWindowHeight", parameters.renderWindowHeight, DEFAULT_PARAMETERS.renderWindowHeight);
    parametersHelper.AddParameter<double>("LinemodTrainParams", "fx", parameters.fx, DEFAULT_PARAMETERS.fx);
    parametersHelper.AddParameter<double>("LinemodTrainParams", "fy", parameters.fy, DEFAULT_PARAMETERS.fy);
    parametersHelper.AddParameter<double>("LinemodTrainParams", "cx", parameters.cx, DEFAULT_PARAMETERS.cx);
    parametersHelper.AddParameter<double>("LinemodTrainParams", "cy", parameters.cy, DEFAULT_PARAMETERS.cy);
    parametersHelper.AddParameter<std::string>("LinemodTrainParams", "CAD object name", parameters.cadObjectName, DEFAULT_PARAMETERS.cadObjectName);
    parametersHelper.AddParameter<bool>("LinemodTrainParams", "Is .ply?", parameters.isPLY, DEFAULT_PARAMETERS.isPLY);
    parametersHelper.AddParameter<bool>("LinemodTrainParams", "Use depth", parameters.useDepthModality, DEFAULT_PARAMETERS.useDepthModality);
    parametersHelper.AddParameter<float>("LinemodTrainParams", "longMin", parameters.longMin, DEFAULT_PARAMETERS.longMin);
    parametersHelper.AddParameter<float>("LinemodTrainParams", "longStep", parameters.longStep, DEFAULT_PARAMETERS.longStep);
    parametersHelper.AddParameter<float>("LinemodTrainParams", "longMax", parameters.longMax, DEFAULT_PARAMETERS.longMax);
    parametersHelper.AddParameter<float>("LinemodTrainParams", "latMin", parameters.latMin, DEFAULT_PARAMETERS.latMin);
    parametersHelper.AddParameter<float>("LinemodTrainParams", "latStep", parameters.latStep, DEFAULT_PARAMETERS.latStep);
    parametersHelper.AddParameter<float>("LinemodTrainParams", "latMax", parameters.latMax, DEFAULT_PARAMETERS.latMax);
    parametersHelper.AddParameter<float>("LinemodTrainParams", "angleMin", parameters.angleMin, DEFAULT_PARAMETERS.angleMin);
    parametersHelper.AddParameter<float>("LinemodTrainParams", "angleStep", parameters.angleStep, DEFAULT_PARAMETERS.angleStep);
    parametersHelper.AddParameter<float>("LinemodTrainParams", "angleMax", parameters.angleMax, DEFAULT_PARAMETERS.angleMax);
    parametersHelper.AddParameter<float>("LinemodTrainParams", "radiusMin", parameters.radiusMin, DEFAULT_PARAMETERS.radiusMin);
    parametersHelper.AddParameter<float>("LinemodTrainParams", "radiusStep", parameters.radiusStep, DEFAULT_PARAMETERS.radiusStep);
    parametersHelper.AddParameter<float>("LinemodTrainParams", "radiusMax", parameters.radiusMax, DEFAULT_PARAMETERS.radiusMax);
    parametersHelper.AddParameter<std::string>("LinemodTrainParams", "saveTrainingImg", parameters.saveTrainingImg, DEFAULT_PARAMETERS.saveTrainingImg);

    configurationFilePath = "";
}

LinemodTrain::~LinemodTrain()
{
}

void LinemodTrain::configure()
{
    if (!configurationFilePath.empty()) {
        parametersHelper.ReadFile(configurationFilePath);
    }
    ValidateParameters();
}

void LinemodTrain::process()
{
    LinemodBasedPoseDetector linemod_detector;
    if (parameters.useDepthModality) {
        linemod_detector.InitAs3D(parameters.T_level0, parameters.T_level1);
        std::cout << " *** 3D LineMOD initialized, T_level0: " << parameters.T_level0
                  << " ; T_level1: " << parameters.T_level1 << " ... " << std::endl;
    } else {
        linemod_detector.InitAs2D(parameters.T_level0, parameters.T_level1);
        std::cout << " *** 2D LineMOD initialized, T_level0: " << parameters.T_level0
                  << " ; T_level1: " << parameters.T_level1 << " ... " << std::endl;
    }

    std::cout << " *** Training ... "<<std::endl;
    int64 t0 = cv::getTickCount();

    //Camera intrinsic parameters
    linemod_detector._winW = parameters.renderWindowWidth;
    linemod_detector._winH = parameters.renderWindowHeight;
    linemod_detector._fx = parameters.fx;
    linemod_detector._fy = parameters.fy;
    linemod_detector._cx = parameters.cx;
    linemod_detector._cy = parameters.cy;
    linemod_detector._saveDir = parameters.saveTrainingImg;

    linemod_detector.Train(parameters.cadObjectName, parameters.isPLY,
                           parameters.longMin, parameters.longMax, parameters.longStep,
                           parameters.latMin, parameters.latMax, parameters.latStep,
                           parameters.angleMin, parameters.angleMax, parameters.angleStep,
                           parameters.radiusMin, parameters.radiusMax, parameters.radiusStep);
    int64 t1 = cv::getTickCount();
    std::cout << " *** Done in "<< (t1-t0)/cv::getTickFrequency() << " seconds" << std::endl;

    linemod_detector.SaveTraining(parameters.cadObjectName);
    std::cout << " *** Training templates saved in "
              << parameters.cadObjectName.substr(0, parameters.cadObjectName.find_last_of("."))
              << "(_training.dat/_poses.dat)" << std::endl;
}

void LinemodTrain::ValidateParameters()
{
    ASSERT(parameters.renderWindowWidth > 0, "Width of the rendering window must be > 0");
    ASSERT(parameters.renderWindowHeight > 0, "Height of the rendering window must be > 0");
    ASSERT(parameters.fx > 0, "Focal length in x must be > 0");
    ASSERT(parameters.fy > 0, "Focal length in y must be > 0");
    ASSERT(parameters.cx >= 0 && parameters.cx < parameters.renderWindowWidth, "Principal point in x must be inside the rendering window");
    ASSERT(parameters.cy >= 0 && parameters.cy < parameters.renderWindowHeight, "Principal point in y must be inside the rendering window");
    ASSERT(parameters.longMax >= parameters.longMin, "Longitude max must be >= longitude min");
    ASSERT(parameters.latMax >= parameters.latMin, "Latitude max must be >= latitude min");
    ASSERT(parameters.angleMax >= parameters.angleMin, "Angle max must be >= angle min");
    ASSERT(parameters.radiusMax >= parameters.radiusMin, "Radius max must be >= radius min");
}

const LinemodTrain::LinemodTrainParams LinemodTrain::DEFAULT_PARAMETERS = {
    //Default parameters for DLR servicer
    .T_level0 = 5,
    .T_level1 = 8,
    .renderWindowWidth = 640,
    .renderWindowHeight = 480,
    .fx = 682.559,
    .fy = 682.261,
    .cx = (288.105 - 264.0) * 640.0/528.0 + 320.0,
    .cy = (225.578 - 203.0) * 480.0/406.0 + 240.0,
    .cadObjectName = "",
    .isPLY = false,
    .useDepthModality = false,
    .longMin = 40.0f,
    .longStep = 1.0f,
    .longMax = 140.0f,
    .latMin = 0.0f,
    .latStep = 1.0f,
    .latMax = 50.0f,
    .angleMin = 0.0f,
    .angleStep = 1.0f,
    .angleMax = 0.0f,
    .radiusMin = 1.0f,
    .radiusStep = 0.1f,
    .radiusMax = 3.0f,
    .saveTrainingImg = ""
};

}
}
}
/** @} */
