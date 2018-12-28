/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGERECTIFICATION_IMAGERECTIFICATIONEDRES_HPP
#define IMAGERECTIFICATION_IMAGERECTIFICATIONEDRES_HPP

#include "ImageRectificationInterface.hpp"

namespace CDFF
{
namespace DFN
{
namespace ImageRectification
{
    /**
     * TODO Class documentation
     */
    class ImageRectificationEdres : public ImageRectificationInterface
    {
        public:

            ImageRectificationEdres();
            virtual ~ImageRectificationEdres();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // IMAGERECTIFICATION_IMAGERECTIFICATIONEDRES_HPP

/** @} */
