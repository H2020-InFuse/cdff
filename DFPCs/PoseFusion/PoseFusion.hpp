/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef POSEFUSION_HPP
#define POSEFUSION_HPP

#include "PoseFusionInterface.hpp"

namespace dfpc_ci
{
    /**
     * TODO Class documentation
     */
    class PoseFusion : public PoseFusionInterface
    {
        public:

            PoseFusion();
            virtual ~PoseFusion();

            virtual void setup();
            virtual void run();

    };
}

#endif // POSEFUSION_HPP

/** @} */
