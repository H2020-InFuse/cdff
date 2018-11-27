/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef VISUALODOMETRY_LAAS_HPP
#define VISUALODOMETRY_LAAS_HPP

#include "VisualOdometry_LAASInterface.hpp"

namespace CDFF
{
namespace DFPC
{
    /**
     * TODO Class documentation
     */
    class VisualOdometry_LAAS : public VisualOdometry_LAASInterface
    {
        public:

            VisualOdometry_LAAS();
            virtual ~VisualOdometry_LAAS();

            virtual void setup();
            virtual void run();

    };
}
}

#endif // VISUALODOMETRY_LAAS_HPP

/** @} */
