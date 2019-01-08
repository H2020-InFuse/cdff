/**
 * @addtogroup DFNs
 * @{
 */

#ifndef DISPARITYFILTERING_DISPARITYFILTERINGEDRES_HPP
#define DISPARITYFILTERING_DISPARITYFILTERINGEDRES_HPP

#include "DisparityFilteringInterface.hpp"

namespace CDFF
{
namespace DFN
{
namespace DisparityFiltering
{
    /**
     * TODO Class documentation
     */
    class DisparityFilteringEdres : public DisparityFilteringInterface
    {
        public:

            DisparityFilteringEdres();
            virtual ~DisparityFilteringEdres();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // DISPARITYFILTERING_DISPARITYFILTERINGEDRES_HPP

/** @} */
