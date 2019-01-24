/**
 * @addtogroup DFPCs
 * @{
 */

#include "WheeledRobotTracker.hpp"

#include <Converters/StdVectorOfStringsToStringSequenceConverter.hpp>
#include <iostream>
#include <fstream>

namespace robots
{
    const std::string SHERPA = "SHERPA";
    const std::string MANA = "MANA";
}

namespace CDFF
{
namespace DFPC
{

WheeledRobotTracker::WheeledRobotTracker()
:     m_robot(robots::MANA)
{
    m_background_subtractor.reset( new CDFF::DFN::ImageFiltering::BackgroundSubtractorMOG2());
    m_circle_finder.reset(new CDFF::DFN::PrimitiveFinder::BasicPrimitiveFinder());
    m_pose_estimator.reset(new CDFF::DFN::PoseEstimator::WheeledRobotPoseEstimator());
}

void WheeledRobotTracker::setup()
{
    std::string path_to_cdff = std::string("");
    if( std::getenv("CDFFPATH") )
    {
        path_to_cdff = std::string(std::getenv("CDFFPATH"));
        if( configurationFilePath.empty() )
        {
            configurationFilePath = std::string(std::getenv("CDFFPATH")) + "/Tests/ConfigurationFiles/DFPCs/ModelBasedTracker/"+getRobotConfigurationFileName();
        }
    }
    else
    {
        path_to_cdff = std::string("../../../");
        if( configurationFilePath.empty() )
        {
            configurationFilePath = "Tests/ConfigurationFiles/DFPCs/ModelBasedTracker/"+getRobotConfigurationFileName();
        }
    }

    std::ifstream configurationFile(configurationFilePath.c_str());
    std::string background_subtractor_config_file, primitive_finder_config_file, pose_estimator_config_file;
    configurationFile >> background_subtractor_config_file;
    configurationFile >> primitive_finder_config_file;
    configurationFile >> pose_estimator_config_file;

    m_background_subtractor->setConfigurationFile(path_to_cdff+background_subtractor_config_file);
    m_background_subtractor->configure();

    m_circle_finder->setConfigurationFile(path_to_cdff+primitive_finder_config_file);
    m_circle_finder->configure();

    m_pose_estimator->setConfigurationFile(path_to_cdff+pose_estimator_config_file);
    m_pose_estimator->configure();
}

std::string WheeledRobotTracker::getRobotConfigurationFileName()
{
    if(m_robot == robots::SHERPA)
    {
        return "DfpcModelBasedTracker_SherpaTracker.yaml";
    }
    else if(m_robot == robots::MANA)
    {
        return "DfpcModelBasedTracker_ManaTracker.yaml";
    }
    else //Return, no config file for given robot.
    {
        std::cerr<<"No config file for requested robot.";
        return std::string();
    }
};

void WheeledRobotTracker::run()
{
    asn1SccPose_Initialize(&outPose);

    //Update setup if the robot name has changed
    std::string robot = std::string(reinterpret_cast<char const*>(inRobotName.arr), inRobotName.nCount);
    if(robot != m_robot)
    {
        m_robot = robot;
        setup();
    }

    m_background_subtractor->imageInput(inImage);
    m_background_subtractor->process();
    const asn1SccFrame & frame_without_background = m_background_subtractor->imageOutput();

    //Circle finder:
    std::string circle_string = "circle";
    asn1SccT_String circle;
    circle.nCount = circle_string.size();
    memcpy(circle.arr, circle_string.data(), circle_string.length());

    m_circle_finder->primitiveInput(circle);
    m_circle_finder->imageInput(frame_without_background);
    m_circle_finder->process();

    m_pose_estimator->depthInput(inDepth);
    m_pose_estimator->primitivesInput(m_circle_finder->primitivesOutput());
    m_pose_estimator->imageInput(frame_without_background);

    m_pose_estimator->process();
    const asn1SccPosesSequence& in_poses = m_pose_estimator->posesOutput();
    if(in_poses.nCount > 0)
    {
        outPose = in_poses.arr[0];
    }
}
}
}

/** @} */
