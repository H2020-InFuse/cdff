/**
 * @addtogroup DFPCs
 *
 * @{
 */

#ifndef VISUALSLAMSTEREO_HPP
#define VISUALSLAMSTEREO_HPP

#include "VisualSlamStereoInterface.hpp"
#include "StereoSlam/StereoSlamInterface.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include <DfpcConfigurator.hpp>


namespace CDFF
{
namespace DFPC
{
/**
 * This DFPC exposes a DFPC interface to the StereoSlamOrb DFN, which is in itself a monolithic DFN
 * encapsulating the whole SLAM functionality.
 * This DFPC is configured according to the StereoSlamOrb DFN parameters only.
 *
 * The input is a Frame Pair coming from the stereo bench.
 * The output is an estimated pose expressed in the reference frame of the first tracked image.
 */
class VisualSlamStereo : public VisualSlamStereoInterface
{
public:

    VisualSlamStereo();
    virtual ~VisualSlamStereo();

    virtual void setup() override;
    virtual void run() override;

private:

    //General configuration helper
    DfpcConfigurator configurator;

    //DFN instantuation method
    void InstantiateDFNs();

    //Pointers to DFN instances
    CDFF::DFN::StereoSlamInterface* slam;

};
}
}

#endif // VISUALSLAMSTEREO_HPP

/** @} */
