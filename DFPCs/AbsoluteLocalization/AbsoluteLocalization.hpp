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

            virtual void setup() override;
            virtual void run() override;

    };
}
}

#endif // ABSOLUTELOCALIZATION_HPP

/** @} */
