/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef VISUALSLAM_HPP
#define VISUALSLAM_HPP

#include "VisualSlamInterface.hpp"

namespace dfpc_ci
{
    /**
     * TODO Class documentation
     */
    class VisualSlam : public VisualSlamInterface
    {
        public:

            VisualSlam();
            virtual ~VisualSlam();

            virtual void setup();
            virtual void run();

    };
}

#endif // VISUALSLAM_HPP

/** @} */
