/**
 * @addtogroup DFNs
 * @{
 */

#ifndef IMAGEPAIRDEGRADATION_IMAGEPAIRDEGRADATION_HPP
#define IMAGEPAIRDEGRADATION_IMAGEPAIRDEGRADATION_HPP

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
    class ImagePairDegradation : public ImagePairDegradationInterface
    {
        public:

            ImagePairDegradation();
            virtual ~ImagePairDegradation();

            virtual void configure();
            virtual void process();
    };
}
}
}

#endif // IMAGEPAIRDEGRADATION_IMAGEPAIRDEGRADATION_HPP

/** @} */
