/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEDEGRADATION_IMAGEDEGRADATION_HPP
#define IMAGEDEGRADATION_IMAGEDEGRADATION_HPP

#include "ImageDegradationInterface.hpp"

namespace CDFF
{
namespace DFN
{
namespace ImageDegradation
{
    /**
     * TODO Class documentation
     */
    class ImageDegradation : public ImageDegradationInterface
    {
        public:

            ImageDegradation();
            virtual ~ImageDegradation();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // IMAGEDEGRADATION_IMAGEDEGRADATION_HPP

/** @} */
