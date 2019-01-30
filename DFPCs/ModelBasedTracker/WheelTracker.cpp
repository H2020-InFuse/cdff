/**
 * @addtogroup DFPCs
 * @{
 */

#include "WheelTracker.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <Converters/StdVectorOfStringsToStringSequenceConverter.hpp>

#include <iostream>
#include <fstream>


namespace CDFF
{
namespace DFPC
{

WheelTracker::WheelTracker()
{
    m_background_subtractor.reset( new CDFF::DFN::ImageFiltering::BackgroundSubtractorMOG2());
    m_ellipse_finder.reset(new CDFF::DFN::PrimitiveFinder::BasicPrimitiveFinder());
    m_circle_finder.reset(new CDFF::DFN::PrimitiveFinder::BasicPrimitiveFinder());
    m_ellipse_pose_estimator.reset(new CDFF::DFN::PoseEstimator::PrimitivesPoseEstimator());
    m_circle_pose_estimator.reset(new CDFF::DFN::PoseEstimator::PrimitivesPoseEstimator());
    m_pose_weighting.reset( new CDFF::DFN::PoseWeighting::KalmanFilter);
}

void WheelTracker::setup()
{
    std::string path_to_cdff = std::string("");
    if( std::getenv("CDFFPATH") )
    {
        path_to_cdff = std::string(std::getenv("CDFFPATH"));
        if( configurationFilePath.empty() )
        {
            configurationFilePath = std::string(std::getenv("CDFFPATH")) + "/Tests/ConfigurationFiles/DFPCs/ModelBasedTracker/DfpcModelBasedTracker_conf01.yaml";
        }
    }
    else
    {
        path_to_cdff = std::string("../../../");
        if( configurationFilePath.empty() )
        {
            configurationFilePath = "Tests/ConfigurationFiles/DFPCs/ModelBasedTracker/DfpcModelBasedTracker_conf01.yaml";
        }
    }

    std::ifstream configurationFile(configurationFilePath.c_str());
    std::string background_subtractor_config_file, primitive_finder_config_file, pose_estimator_config_file, pose_weighting_config_file;
    configurationFile >> background_subtractor_config_file;
    configurationFile >> primitive_finder_config_file;
    configurationFile >> pose_estimator_config_file;
    configurationFile >> pose_weighting_config_file;

    m_background_subtractor->setConfigurationFile(path_to_cdff+background_subtractor_config_file);
    m_background_subtractor->configure();

    m_ellipse_finder->setConfigurationFile(path_to_cdff+primitive_finder_config_file);
    m_ellipse_finder->configure();

    m_circle_finder->setConfigurationFile(path_to_cdff+primitive_finder_config_file);
    m_circle_finder->configure();

    m_ellipse_pose_estimator->setConfigurationFile(path_to_cdff+pose_estimator_config_file);
    m_ellipse_pose_estimator->configure();

    m_circle_pose_estimator->setConfigurationFile(path_to_cdff+pose_estimator_config_file);
    m_circle_pose_estimator->configure();

    m_pose_weighting->setConfigurationFile(path_to_cdff+pose_weighting_config_file);
    m_pose_weighting->configure();
}

void WheelTracker::run()
{
    m_background_subtractor->imageInput(inImage);
    m_background_subtractor->process();
    const asn1SccFrame & frame_without_background = m_background_subtractor->imageOutput();

    //Ellipse finder:
    std::string primitives_string = "ellipse";
    asn1SccT_String primitive;
    primitive.nCount = primitives_string.size();
    memcpy(primitive.arr, primitives_string.data(), primitives_string.length());

    m_ellipse_finder->primitiveInput(primitive);
    m_ellipse_finder->imageInput(frame_without_background);
    m_ellipse_finder->process();

    m_ellipse_pose_estimator->imageInput(frame_without_background);
    m_ellipse_pose_estimator->primitivesInput(m_ellipse_finder->primitivesOutput());
    m_ellipse_pose_estimator->depthInput(inDepth);
    m_ellipse_pose_estimator->process();


    //Circle finder:
    std::string circle_string = "circle";
    asn1SccT_String circle;
    circle.nCount = circle_string.size();
    memcpy(circle.arr, circle_string.data(), circle_string.length());

    m_circle_finder->primitiveInput(circle);
    m_circle_finder->imageInput(frame_without_background);
    m_circle_finder->process();

    m_circle_pose_estimator->imageInput(frame_without_background);
    m_circle_pose_estimator->primitivesInput(m_circle_finder->primitivesOutput());
    m_circle_pose_estimator->depthInput(inDepth);
    m_circle_pose_estimator->process();



    //Weighting algorithm:
    asn1SccPosesSequence poses;
    poses.nCount = 2;
    poses.arr[0] = m_circle_pose_estimator->posesOutput().arr[0];
    poses.arr[1] = m_ellipse_pose_estimator->posesOutput().arr[0];

    m_pose_weighting->posesInput(poses);
    m_pose_weighting->process();

    outPose = m_pose_weighting->poseOutput();
}

}
}

/** @} */
