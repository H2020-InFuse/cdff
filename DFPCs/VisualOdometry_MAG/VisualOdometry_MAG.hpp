/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef VISUALODOMETRY_MAG_HPP
#define VISUALODOMETRY_MAG_HPP

#include "VisualOdometry_MAGInterface.hpp"

namespace CDFF
{
namespace DFPC
{
    /**
     * TODO Class documentation
     */
    class VisualOdometry_MAG : public VisualOdometry_MAGInterface
    {
        public:

            VisualOdometry_MAG();
            virtual ~VisualOdometry_MAG();

            virtual void setup() override;
            virtual void run() override;

    };
}
}

#endif // VISUALODOMETRY_MAG_HPP

/** @} */
