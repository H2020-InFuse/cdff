/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGERECTIFICATION_IMAGERECTIFICATION_HPP
#define IMAGERECTIFICATION_IMAGERECTIFICATION_HPP

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
    class ImageRectification : public ImageRectificationInterface
    {
        public:

            ImageRectification();
            virtual ~ImageRectification();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // IMAGERECTIFICATION_IMAGERECTIFICATION_HPP

/** @} */
