/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEPAIRDEGRADATION_IMAGEPAIRDEGRADATIONEDRES_HPP
#define IMAGEPAIRDEGRADATION_IMAGEPAIRDEGRADATIONEDRES_HPP

#include "ImagePairDegradationInterface.hpp"

namespace CDFF
{
namespace DFN
{
namespace ImagePairDegradation
{
    /**
     * TODO Class documentation
     */
    class ImagePairDegradationEdres : public ImagePairDegradationInterface
    {
        public:

            ImagePairDegradationEdres();
            virtual ~ImagePairDegradationEdres();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // IMAGEPAIRDEGRADATION_IMAGEPAIRDEGRADATIONEDRES_HPP

/** @} */
