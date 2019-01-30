/**
 * @addtogroup DFPCs
 * @{
 */

#include "VisualSlamStereo.hpp"

namespace CDFF
{
namespace DFPC
{

VisualSlamStereo::VisualSlamStereo()
{
    slam = nullptr;
    configurationFilePath = "";
}

VisualSlamStereo::~VisualSlamStereo()
{
}

void VisualSlamStereo::setup()
{
    configurator.configure(configurationFilePath);

    // Create and configure DFNs
    InstantiateDFNs();
}

void VisualSlamStereo::run()
{
    ASSERT( slam!= nullptr, "VisualSlamStereo, Slam DFN is null");
    slam->framePairInput(inFramePair);
    slam->process();
    outEstimatedPose = slam->poseOutput();
}


void VisualSlamStereo::InstantiateDFNs()
    {
    slam = static_cast<CDFF::DFN::StereoSlamInterface*>(configurator.GetDfn("stereoSlamOrb"));
    }

}
}

/** @} */
