/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef LIDARMAPBASEDLOCALIZATION_HPP
#define LIDARMAPBASEDLOCALIZATION_HPP

#include "LIDARMapBasedLocalizationInterface.hpp"

namespace CDFF
{
namespace DFPC
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
}

#endif // LIDARMAPBASEDLOCALIZATION_HPP

/** @} */
