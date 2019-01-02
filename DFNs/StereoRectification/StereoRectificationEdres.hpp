/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREORECTIFICATION_STEREORECTIFICATIONEDRES_HPP
#define STEREORECTIFICATION_STEREORECTIFICATIONEDRES_HPP

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
    class StereoRectificationEdres : public StereoRectificationInterface
    {
        public:

            StereoRectificationEdres();
            virtual ~StereoRectificationEdres();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // STEREORECTIFICATION_STEREORECTIFICATIONEDRES_HPP

/** @} */
