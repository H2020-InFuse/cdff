/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef VISUALSLAMSTEREO_HPP
#define VISUALSLAMSTEREO_HPP

#include "VisualSlamStereoInterface.hpp"

namespace CDFF
{
namespace DFPC
{
    /**
     * TODO Class documentation
     */
    class VisualSlamStereo : public VisualSlamStereoInterface
    {
        public:

            VisualSlamStereo();
            virtual ~VisualSlamStereo();

            virtual void setup();
            virtual void run();

    };
}
}

#endif // VISUALSLAMSTEREO_HPP

/** @} */
