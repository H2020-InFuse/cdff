/**
 * @addtogroup DFNs
 * @{
 */

#include "StereoSlamOrb.hpp"

namespace CDFF
{
namespace DFN
{
namespace StereoSlam
{

StereoSlamOrb::StereoSlamOrb()
{
    parameters = DEFAULT_PARAMETERS;

    parametersHelper.AddParameter<std::string>("GeneralParameters", "SettingsPath", parameters.settingsPath, DEFAULT_PARAMETERS.settingsPath);
    parametersHelper.AddParameter<std::string>("GeneralParameters", "VocabularyPath", parameters.vocabularyPath, DEFAULT_PARAMETERS.vocabularyPath);
    parametersHelper.AddParameter<bool>("GeneralParameters", "EnableViewer", parameters.enableViewer, DEFAULT_PARAMETERS.enableViewer);
    parametersHelper.AddParameter<bool>("GeneralParameters", "SaveMap", parameters.saveMap, DEFAULT_PARAMETERS.saveMap);
    parametersHelper.AddParameter<std::string>("GeneralParameters", "MapFile", parameters.mapFile, DEFAULT_PARAMETERS.mapFile);

    configurationFilePath = "";
}

StereoSlamOrb::~StereoSlamOrb()
{
}

void StereoSlamOrb::configure()
{
    // TODO DFN configuration steps
    parametersHelper.ReadFile(configurationFilePath);
    ValidateParameters();
}

void StereoSlamOrb::process()
{
    // TODO DFN processing steps
}

const StereoSlamOrb::StereoSlamOrbOptionsSet StereoSlamOrb::DEFAULT_PARAMETERS =
{
    /*.settingPath =*/ "",
    /*.vocabularyPath =*/ "",
    /*.enableViewer =*/ false,
    /*.saveMap =*/ false,
    /*.mapFile =*/ ""
};

void StereoSlamOrb::ValidateParameters()
{
    ASSERT(parameters.settingsPath.size() > 0, "StereoSlamOrb Configuration Error: Orb-Slam settings file path must be specified");
    ASSERT(parameters.vocabularyPath.size() > 0, "StereoSlamOrb Configuration Error: Orb-Slam vocabulary file path must be specified");
    if(parameters.saveMap)
    {
        ASSERT(parameters.mapFile.size() > 0, "StereoSlamOrb Configuration Error: A path to the previously generated map file must be specified");
    }
}

void StereoSlamOrb::ValidateInputs(const asn1SccFramePair& pair)
{
}

}
}
}

/** @} */
