/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef VISUALODOMETRY_MAG_HPP
#define VISUALODOMETRY_MAG_HPP

#include "VisualOdometry_MAGInterface.hpp"

namespace dfpc_ci
{
    /**
     * TODO Class documentation
     */
    class VisualOdometry_MAG : public VisualOdometry_MAGInterface
    {
        public:

            VisualOdometry_MAG();
            virtual ~VisualOdometry_MAG();

            virtual void setup();
            virtual void run();

    };
}

#endif // VISUALODOMETRY_MAG_HPP

/** @} */
