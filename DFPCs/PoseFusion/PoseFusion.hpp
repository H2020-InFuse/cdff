/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef POSEFUSION_HPP
#define POSEFUSION_HPP

#include "PoseFusionInterface.hpp"

namespace CDFF
{
namespace DFPC
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
}

#endif // POSEFUSION_HPP

/** @} */
