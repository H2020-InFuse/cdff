/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREORECTIFICATION_STEREORECTIFICATION_HPP
#define STEREORECTIFICATION_STEREORECTIFICATION_HPP

#include "StereoRectificationInterface.hpp"

namespace CDFF
{
namespace DFN
{
namespace StereoRectification
{
    /**
     * TODO Class documentation
     */
    class StereoRectification : public StereoRectificationInterface
    {
        public:

            StereoRectification();
            virtual ~StereoRectification();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // STEREORECTIFICATION_STEREORECTIFICATION_HPP

/** @} */
