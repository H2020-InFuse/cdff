/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef ABSOLUTELOCALIZATION_HPP
#define ABSOLUTELOCALIZATION_HPP

#include "AbsoluteLocalizationInterface.hpp"

namespace CDFF
{
namespace DFPC
{
    /**
     * TODO Class documentation
     */
    class AbsoluteLocalization : public AbsoluteLocalizationInterface
    {
        public:

            AbsoluteLocalization();
            virtual ~AbsoluteLocalization();

            virtual void setup();
            virtual void run();

    };
}
}

#endif // ABSOLUTELOCALIZATION_HPP

/** @} */
