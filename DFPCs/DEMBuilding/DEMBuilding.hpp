/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef DEMBUILDING_HPP
#define DEMBUILDING_HPP

#include "DEMBuildingInterface.hpp"

namespace dfpc_ci
{
    /**
     * TODO Class documentation
     */
    class DEMBuilding : public DEMBuildingInterface
    {
        public:

            DEMBuilding();
            virtual ~DEMBuilding();

            virtual void setup();
            virtual void run();

    };
}

#endif // DEMBUILDING_HPP

/** @} */
