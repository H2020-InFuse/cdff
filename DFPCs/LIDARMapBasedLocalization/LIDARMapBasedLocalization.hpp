/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef LIDARMAPBASEDLOCALIZATION_HPP
#define LIDARMAPBASEDLOCALIZATION_HPP

#include "LIDARMapBasedLocalizationInterface.hpp"

namespace dfpc_ci
{
    /**
     * TODO Class documentation
     */
    class LIDARMapBasedLocalization : public LIDARMapBasedLocalizationInterface
    {
        public:

            LIDARMapBasedLocalization();
            virtual ~LIDARMapBasedLocalization();

            virtual void setup();
            virtual void run();

    };
}

#endif // LIDARMAPBASEDLOCALIZATION_HPP

/** @} */
