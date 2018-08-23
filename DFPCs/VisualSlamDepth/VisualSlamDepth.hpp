/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef VISUALSLAMDEPTH_HPP
#define VISUALSLAMDEPTH_HPP

#include "VisualSlamDepthInterface.hpp"

namespace CDFF
{
namespace DFPC
{
    /**
     * TODO Class documentation
     */
    class VisualSlamDepth : public VisualSlamDepthInterface
    {
        public:

            VisualSlamDepth();
            virtual ~VisualSlamDepth();

            virtual void setup();
            virtual void run();

    };
}
}

#endif // VISUALSLAMDEPTH_HPP

/** @} */
