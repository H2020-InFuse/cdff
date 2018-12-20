/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEDEGRADATION_IMAGEDEGRADATIONEDRES_HPP
#define IMAGEDEGRADATION_IMAGEDEGRADATIONEDRES_HPP

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
    class ImageDegradationEdres : public ImageDegradationInterface
    {
        public:

            ImageDegradationEdres();
            virtual ~ImageDegradationEdres();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // IMAGEDEGRADATION_IMAGEDEGRADATIONEDRES_HPP

/** @} */
